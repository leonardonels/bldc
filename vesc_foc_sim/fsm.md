# Motor FSM — VESC firmware (FOC path)

## TL;DR — one-page mental model

Two variables define everything:

- `m_state`   — is the inverter **on** or **off**? (OFF / RUNNING)
- `m_control_mode` — while RUNNING, **what is it tracking**? (duty, speed,
  current, brake, handbrake, position, openloop…)

Every CAN setter (`SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_RPM`, …) does the
same three things: it picks a mode, writes a setpoint, and if the setpoint is
large enough it turns the bridge on. No blending — **last command wins**. If
all setpoints fall below `cc_min_current` and the off-delay expires, the
bridge turns off (coast).

Quick picker:

| You want…                                             | Send                                  | Resulting mode            |
|-------------------------------------------------------|---------------------------------------|---------------------------|
| Accelerate / decelerate with a signed torque          | `SET_CURRENT(iq)`                     | `CURRENT`                 |
| Pure deceleration opposing motion (any direction)     | `SET_CURRENT_BRAKE(\|iq\|)`             | `CURRENT_BRAKE`           |
| Hold the rotor still at its current angle             | `SET_CURRENT_HANDBRAKE(iq)`           | `HANDBRAKE`               |
| Freewheel — zero shaft torque, don't load the ICE     | `SET_CURRENT(0)` + off_delay (§6)     | `CURRENT` with iq=0       |
| Fully coast, inverter off                             | `SET_CURRENT(~0)` until release fires | `OFF` / `NONE`            |
| Target a speed / position                             | `SET_RPM` / `SET_POS`                 | `SPEED` / `POS`           |

Simple diagram:

```
                    bridge off, phases high-Z (coast)
                 ┌──────────────── OFF ────────────────┐
                 │                                     │
                 │  any setter with |setpoint| ≥ Imin  │
                 │                 ▼                   │ |setpoint|<Imin
                 │           ┌──────────┐              │ AND off_delay==0
                 │           │ RUNNING  │──────────────┘   (or timeout / fault)
                 │           └──────────┘
                 │       selects one mode:
                 │    CURRENT  ◄──►  CURRENT_BRAKE
                 │         ▲    ╲   ╱    ▲
                 │         │     ╲ ╱     │
                 │         ▼      X      ▼
                 │    HANDBRAKE  ╱ ╲  DUTY / SPEED / POS / OPENLOOP*
                 │              ╱   ╲
                 │   (any mode ↔ any mode via last-writer-wins)
                 └─────────────────────────────────────────────
```

Three behaviours worth remembering:

1. **`CURRENT`** gives you a signed q-current. Negative = regen; at ω=0 a
   negative iq will start spinning the motor backwards. Good for torque
   control, not for "stop and stay".
2. **`CURRENT_BRAKE`** re-computes iq as `−SIGN(ω)·|iq_set|` every tick.
   Always opposes motion, never drives. At ω=0 the commanded iq collapses to
   zero, so it is a **velocity regulator, not a position hold**.
3. **`HANDBRAKE`** injects a static current vector at a fixed electrical
   angle, producing a restoring torque that is a function of rotor **angle**.
   It is the only mode that actually clamps position.

See §6 for the freewheel/hybrid-ICE case (which uses `CURRENT` with iq=0,
not `OFF`, to avoid uncontrolled body-diode rectification at high speed).

---

This document describes the two concurrent state variables that together define
what the inverter is doing, how external commands move between states, and why
each transition exists. It is written from the FOC code path
(`motor/mcpwm_foc.c` + `motor/mc_interface.c`); the BLDC path has a few extra
states (`MC_STATE_DETECTING`, `MC_STATE_FULL_BRAKE`) that the FOC path does not
use.

The FSM is actually **two orthogonal variables**:

- `m_state` — the hardware-level state of the PWM bridge.
- `m_control_mode` — what setpoint the RUNNING state is tracking.

Most "modes" people talk about (current, brake, handbrake, speed, position…)
live in `m_control_mode`. `m_state` is almost binary in FOC: OFF or RUNNING.

---

## 1. `m_state` — hardware state (`mc_state` in `datatypes.h`)

| Value                | FOC uses it? | Meaning |
|----------------------|--------------|---------|
| `MC_STATE_OFF`       | yes          | Bridge disabled, all six FETs off, phases high-Z (coasting). |
| `MC_STATE_DETECTING` | no (BLDC)    | Parameter detection commutation; FOC uses a different path. |
| `MC_STATE_RUNNING`   | yes          | Bridge active, FOC loop enforcing `m_control_mode`. |
| `MC_STATE_FULL_BRAKE`| no (BLDC)    | BLDC-only: all low-side FETs on permanently. FOC achieves the equivalent via duty=0 short-brake inside CURRENT_BRAKE. |

So in FOC the rotor is in exactly one of two hardware situations:

- **OFF** — freewheeling. No torque applied by the inverter. External torques
  move the rotor freely; back-EMF is open-circuit.
- **RUNNING** — the FOC loop at 20 kHz imposes the q/d currents demanded by
  `m_control_mode`.

---

## 2. `m_control_mode` (`mc_control_mode`)

Only meaningful while `m_state == MC_STATE_RUNNING`. Each value selects a
different control law in the fast loop.

| Mode                          | Setpoint(s) written           | Fast-loop behaviour |
|-------------------------------|-------------------------------|---------------------|
| `CONTROL_MODE_NONE`           | —                             | Paired with `MC_STATE_OFF`; no control. |
| `CONTROL_MODE_DUTY`           | `m_duty_cycle_set`            | Open-loop duty on the q-axis via SVM. |
| `CONTROL_MODE_SPEED`          | `m_speed_pid_set_rpm`         | Outer RPM PID → q-current inner loop. |
| `CONTROL_MODE_CURRENT`        | `m_iq_set`, `m_id_set`        | Signed q/d current control. Sign of iq defines direction of torque. |
| `CONTROL_MODE_CURRENT_BRAKE`  | `m_iq_set` (magnitude only)   | Brake: q-current is forced to `-SIGN(ω)·|iq_set|` every tick. Always opposes motion; zero at ω=0. |
| `CONTROL_MODE_POS`            | `m_pos_pid_set`               | Position PID. |
| `CONTROL_MODE_HANDBRAKE`      | `m_iq_set` (d-axis injection) | Open-loop fixed current vector at a static electrical angle → position-dependent restoring torque. |
| `CONTROL_MODE_OPENLOOP*`      | current + speed/phase         | Open-loop rotating or static field. Used for start-up, detection, stimulus. |

---

## 3. Transitions — which CAN / comm command moves where

All current-family commands come in via CAN (`comm/comm_can.c`) or
UART/USB (`comm/commands.c`) and funnel through `mc_interface_*` into the
`mcpwm_foc_set_*` setters. Each setter is the **only** place that writes
`m_control_mode`; there is no merge layer, so sequential commands
**overwrite** each other (last-writer-wins).

Notation: `|I|` is the magnitude of the commanded current; `I_min =
cc_min_current` is the release threshold from motor config.

### From `MC_STATE_OFF` (idle)

Any of the setters below, if `|I| ≥ I_min`, sets
`m_motor_released = false` and transitions to `MC_STATE_RUNNING` with the
corresponding `m_control_mode`.

| Incoming command                            | Goes to                                              | Why |
|---------------------------------------------|------------------------------------------------------|-----|
| `CAN_PACKET_SET_DUTY`                       | RUNNING + `CONTROL_MODE_DUTY`                        | Request raw duty. |
| `CAN_PACKET_SET_CURRENT` (signed)           | RUNNING + `CONTROL_MODE_CURRENT`                     | Signed q-current; sign selects direction, negatives are regen. |
| `CAN_PACKET_SET_CURRENT_BRAKE`              | RUNNING + `CONTROL_MODE_CURRENT_BRAKE`               | Brake magnitude; sign taken from speed, not command. |
| `CAN_PACKET_SET_CURRENT_HANDBRAKE`          | RUNNING + `CONTROL_MODE_HANDBRAKE`                   | Static holding torque at current angle. |
| `CAN_PACKET_SET_RPM`                        | RUNNING + `CONTROL_MODE_SPEED`                       | Speed PID target. |
| `CAN_PACKET_SET_POS`                        | RUNNING + `CONTROL_MODE_POS`                         | Position PID target. |
| `SET_CURRENT_REL` / `_BRAKE_REL` / `_HANDBRAKE_REL` | same as absolute variants                     | Scale by configured max current. |
| `SET_CURRENT` with `\|I\| < I_min`            | stays OFF (or drops to OFF from RUNNING)             | See "implicit release" below. |

### From `MC_STATE_RUNNING`

The transitions below are **mode swaps within RUNNING**; `m_state` does not
change.

| From mode → To mode                          | Trigger                                | Notes |
|----------------------------------------------|----------------------------------------|-------|
| `CURRENT` → `CURRENT_BRAKE`                  | `SET_CURRENT_BRAKE`                    | New setter overwrites mode + `m_iq_set`. Any signed drive command is dropped. |
| `CURRENT_BRAKE` → `CURRENT`                  | `SET_CURRENT`                          | Same mechanism in reverse. No hysteresis, no gradual handoff. |
| `CURRENT` / `CURRENT_BRAKE` → `HANDBRAKE`    | `SET_CURRENT_HANDBRAKE`                | Needed to actually *hold* at ω=0 (brake alone cannot, see §4). |
| `HANDBRAKE` → `CURRENT`                      | `SET_CURRENT`                          | Re-enter normal closed-loop current control. |
| any → `DUTY` / `SPEED` / `POS`               | respective setters                     | Same pattern; each writes its own mode. |
| any → `OPENLOOP*`                            | openloop setters (`mcpwm_foc_set_openloop*`) | Used for start-up / detection stimulus. |

### From `RUNNING` back to `OFF` — release path

There is no explicit "stop" command in CAN. Release is always **implicit** and
evaluated every fast-loop tick at `mcpwm_foc.c` ≈ L3925-3948:

```
if (m_state == RUNNING
    && m_control_mode ∈ {CURRENT, CURRENT_BRAKE, HANDBRAKE, OPENLOOP, OPENLOOP_PHASE}
    && |m_iq_set| < min_current
    && |m_id_set| < min_current
    &&  m_i_fw_set < min_current
    &&  m_current_off_delay < dt) {
        m_control_mode = NONE;
        m_state        = OFF;
        stop_pwm_hw();
}
```

Ways to trigger it:

- **CAN `SET_CURRENT` with `|I| < I_min`** — the value falls through the
  setter's early return and the loop body above drops the bridge.
- **CAN `SET_CURRENT_BRAKE` with `|I| < I_min`** — same idea.
- **`mc_interface_release_motor` / `mcpwm_foc_release_motor`** — sets
  `m_iq_set = m_id_set = 0`, `m_motor_released = true`; the loop then cuts the
  bridge next tick.
- **Timeout** (the watchdog refreshed by `timeout_reset()` in every RX handler)
  — if no command arrives within the configured timeout, `mc_interface` calls
  the timeout handler which releases the motor. This is why every CAN command
  must be repeated at ≥ the timeout rate.
- **Fault** — any detected fault forces state OFF and disables the bridge
  until cleared.

`m_current_off_delay` (optional extra field in the 6-byte `SET_CURRENT`
payload) stretches this release window: the bridge stays RUNNING with iq=0
for the configured delay to avoid chatter if the setpoint briefly dips
through zero.

---

## 4. Why the behaviour feels the way it does

These are consequences of the above tables, written out so the control-side
semantics are obvious.

### 4.1 `SET_CURRENT` vs `SET_CURRENT_BRAKE`

- `SET_CURRENT`: a **signed** q-current. Negative values produce regen torque
  opposing the current rotor direction — the motor brakes **and** delivers
  energy to the DC bus, but if the rotor crosses zero speed while iq is still
  negative, the torque will then accelerate it backwards. Good for precise
  torque control; bad as a "please just stop" command.
- `SET_CURRENT_BRAKE`: the firmware re-computes iq every tick as
  `-SIGN(ω)·|iq_set|` ([mcpwm_foc.c:3425](../motor/mcpwm_foc.c#L3425)). Always opposes
  motion, never drives the motor. Clamped against `lo_current_min`, not
  `lo_current_max` ([mcpwm_foc.c:3302](../motor/mcpwm_foc.c#L3302)). Around sign flips
  and near duty=0 the fast loop switches to passive short-brake (duty=0, all
  low-side on) for ≥10 cycles before returning to current control
  ([mcpwm_foc.c:3323-3360](../motor/mcpwm_foc.c#L3323-L3360)), to smooth the handoff
  between active and passive braking.

Sending both alternately has no blending effect: whichever setter ran last
wins that tick.

### 4.2 Behaviour of `CURRENT_BRAKE` at ω ≈ 0

At exactly ω=0 the sign rule yields iq=0 → no torque; infinitesimal motion
then triggers the full commanded brake current in opposition. So the rotor
is:

- **Not held** positionally — slow external torques can drift it.
- Strongly **velocity-regulated** — any attempt to rotate meets full brake
  torque immediately.

This is why the behaviour around standstill looks like a zero-width dead-band
with bang-bang hold, not a viscous clutch. The min-RPM hold logic that
CURRENT mode uses is explicitly disabled in CURRENT_BRAKE
([mcpwm_foc.c:4064](../motor/mcpwm_foc.c#L4064)) — the firmware deliberately lets the
rotor idle at zero instead of trying to clamp a position.

### 4.3 Why `HANDBRAKE` exists separately

`CURRENT_BRAKE` cannot produce position-holding torque because its control
law is a function of **speed**. `HANDBRAKE` injects an **open-loop d-axis
current vector at a static electrical angle** ([mcpwm_foc.c:852](../motor/mcpwm_foc.c#L852)):
the resulting torque becomes a function of rotor **angle** (detent torque
around the injected vector), so the rotor is actively restored to a specific
position. This is the only built-in mode that holds the motor stationary.

It must be commanded explicitly — nothing in the driver auto-transitions
from brake into handbrake as the rotor slows down. If you want "brake to
zero, then hold", the application layer has to monitor speed and issue the
handbrake command itself.

### 4.4 Why every setter does the same two-field write

Every `mcpwm_foc_set_*` writes, in order:

```
m_control_mode = <new mode>;
m_iq_set        = <value>;
[ m_id_set / m_duty_cycle_set / m_speed_pid_set / m_pos_pid_set / ... ]
if (|setpoint| < cc_min_current) return;   // leave state alone → release path can fire
if (m_state != RUNNING) {
    m_motor_released = false;
    m_state = RUNNING;
}
```

Consequences:

- All mode changes are **instantaneous**; there is no cross-fade.
- There is no layered / merged command model — the last command wins.
- Releasing the motor is always achieved by sending **any** of those commands
  with `|setpoint| < cc_min_current`, or by letting the timeout elapse.

---

## 5. Condensed state diagram

```
                 ┌──────────────────────────────────────────────────────┐
                 │                      MC_STATE_OFF                    │
                 │                m_control_mode = NONE                 │
                 │           (bridge disabled, phases high-Z)           │
                 └───────────┬──────────────────────────────────┬───────┘
   any setter with |I|≥Imin  │                                  │   timeout / release /
                             ▼                                  │   |I|<Imin & off_delay==0
                 ┌──────────────────────────────────────────────┴───────┐
                 │                    MC_STATE_RUNNING                  │
                 │                                                      │
                 │  m_control_mode ∈ {                                  │
                 │     DUTY, SPEED, CURRENT, CURRENT_BRAKE,             │
                 │     POS, HANDBRAKE, OPENLOOP*                        │
                 │  }                                                   │
                 │                                                      │
                 │   Mode swaps via last-writer-wins:                   │
                 │   SET_CURRENT ↔ SET_CURRENT_BRAKE ↔ SET_HANDBRAKE    │
                 │   ↔ SET_DUTY ↔ SET_RPM ↔ SET_POS                     │
                 └──────────────────────────────────────────────────────┘
```

Fault handling (not shown) can force OFF from anywhere.

---

## 6. Freewheel case — hybrid powertrain, no regen, no engine braking

Scenario: the rotor is bolted to the shaft of an ICE-based hybrid powertrain.
The engine (or the wheels, depending on topology) drives the rotor. You want
the electric machine to add **zero** mechanical load to the shaft — not
charging the battery, not braking the engine — until you explicitly ask for
drive or regen.

There are two ways to get zero shaft torque from the inverter, and only one
is safe at arbitrary rotor speed.

### Option A — bridge OFF (coast)

Send `SET_CURRENT` with `|I| < cc_min_current`, or call
`mc_interface_release_motor`. The release path at
[mcpwm_foc.c:3925-3948](../motor/mcpwm_foc.c#L3925-L3948) drives `m_state →
OFF` and disables PWM. All six FETs are off; phases are high-impedance.

Pros: zero electromagnetic torque, zero battery draw, zero switching losses.
Cons: the MOSFET body diodes still form a 3-phase rectifier. When the peak
line-to-line back-EMF exceeds the DC-bus voltage plus two diode drops, the
diodes conduct and the rotor sees an uncontrolled regen torque into the
battery that you cannot turn off. Threshold (approximate, SPMSM):

```
ω_m  >  (V_bus + 2·V_d) / (√3 · p · λ)
```

If the ICE can ever drag the rotor above that threshold, Option A is not
safe for this use case.

### Option B — RUNNING with iq = 0 (recommended)

Send signed `SET_CURRENT` with `iq = 0` plus a non-zero `m_current_off_delay`
(the 6-byte payload variant at [comm_can.c:523-529](../comm/comm_can.c#L523-L529)).
Mode stays `CONTROL_MODE_CURRENT`, `m_state` stays `RUNNING`, the FOC loop
actively regulates id = iq = 0.

Why this gives zero shaft torque **and** zero rectification:

- Electromagnetic torque `τe = (3/2)·p·(λ + (Ld−Lq)·id)·iq = 0` when iq = 0
  and id = 0.
- The FOC loop drives `Vq ≈ ω_e·λ` and `Vd ≈ 0` to cancel the back-EMF
  exactly. No body-diode conduction, regardless of speed.
- Bus current is only switching + gate-drive loss (small, typically aux-rail
  fed). Effectively no traction-battery charge or discharge.
- Transitioning to drive (iq > 0) or regen (iq < 0) takes one control tick —
  no re-arm delay.

Why the off_delay matters: without it, `|iq|=0 < cc_min_current` would trip
the release path next tick and the bridge would fall to OFF. Set
`off_delay` comfortably larger than your command period (e.g. 0.5 s at
≥ 2 Hz refresh). The release check `m_current_off_delay < dt` then never
fires.

Do **not** use `SET_CURRENT_BRAKE` for this: its sign rule produces an
opposing torque the moment the shaft moves — exactly the engine-braking
behaviour you are trying to avoid.

### Cheat sheet

| Goal                                  | Command                                         | Mode / state             |
|---------------------------------------|-------------------------------------------------|--------------------------|
| Freewheel (hybrid idle, ICE unloaded) | `SET_CURRENT { iq=0, off_delay=0.5 }` @ ≥ 2 Hz  | RUNNING + `CURRENT`, iq=0 |
| Consume `P` watts from the shaft      | `SET_CURRENT { iq = P / (1.5·ω_e·λ) }`          | RUNNING + `CURRENT`      |
| Regen `P` watts into the battery      | `SET_CURRENT { iq = −P / (1.5·ω_e·λ) }`         | RUNNING + `CURRENT`      |
| Full coast, inverter disarmed         | `SET_CURRENT { iq ≈ 0, off_delay = 0 }`         | OFF + `NONE`             |

Freewheel diagram:

```
   ICE shaft torque  ─────────►  rotor  ─────────►  shaft torque out
                                   ▲
                                   │ τe = 0   (enforced by FOC @ iq=id=0)
                                   │ Vq ≈ ω_e·λ, Vd ≈ 0  (cancels back-EMF,
                                   │                      no diode conduction)
                              ┌────┴────┐
                              │ RUNNING │   m_control_mode = CURRENT
                              │         │   m_iq_set       = 0
                              │  iq = 0 │   off_delay      > 0  ← prevents auto-release
                              └─────────┘
                            (kept alive by periodic SET_CURRENT @ ≥ 2 Hz)
```

---

## 7. Practical notes for the ROS2 sim

The plant node [pmsm_plant_node.cpp](src/pmsm_plant_node.cpp) only integrates
the electrical ODE driven by (vd, vq). It has **no awareness of
`m_control_mode` or `m_state`**; whoever produces vd/vq upstream is
responsible for reproducing:

- the sign rule of `CURRENT_BRAKE` (`-SIGN(ω)·|iq_set|`),
- the brake-current clamp against `lo_current_min`,
- the duty=0 short-brake handoff around zero speed/sign flips,
- the release path (zero the setpoint below `cc_min_current`, stop driving
  the plant).

For `HANDBRAKE` the upstream node must inject an open-loop static current
vector (not feed it through a Park transform based on rotor angle — the
whole point is that the vector is *fixed* in the stator frame, so the
developed torque depends on the rotor angle).
