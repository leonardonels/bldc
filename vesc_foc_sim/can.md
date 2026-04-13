# CAN Messages — what each setter does to the motor

Companion to [fsm.md](fsm.md). This document assumes you already know the
VESC CAN message **names** (`SET_CURRENT`, `SET_RPM`, …) and want to know,
for each one: *what does the motor do when I send it, what parameters
matter, and when should I pick it over a neighbour?*

No byte layouts, no endianness, no DLC tables — those belong in the VESC
protocol reference. Here we talk behaviour.

## 0. Shared rules (true for every setter below)

- **Target.** Each message is addressed to a single VESC by its
  `controller_id`. A broadcast form exists; avoid it for setpoints.
- **No arm, no disarm, no mode packet.** The message *name* selects the
  control mode. The first message with a non-trivial setpoint arms the
  bridge; absence of messages disarms it.
- **Last-writer-wins.** Sending `SET_CURRENT` after `SET_CURRENT_BRAKE`
  swaps the mode instantly — no blend, no handoff, no "stop" needed in
  between.
- **You must repeat.** Every setter refreshes a watchdog. If nothing
  arrives within the configured timeout (typically 1 s), the firmware
  auto-releases the motor → bridge OFF, phases high-Z. Normal cadence is
  ≥ 10 Hz; the freewheel case (`SET_CURRENT` with `iq=0`) tolerates ≥ 2 Hz
  because it carries its own `off_delay`.
- **Units on the wire are absolute (A, RPM, degrees).** The `_REL`
  variants carry a fraction of the configured max motor current, so the
  tuning lives in the firmware config rather than in your node.

## 1. The setters, one by one

### `SET_DUTY`

Open-loop duty-cycle command on the q-axis. Engages `CONTROL_MODE_DUTY`.

- **Parameter:** duty ∈ [−1.0, +1.0].
- **Use when:** bench testing, characterisation, or you explicitly want to
  bypass the current loop. Global motor current limits still apply, but
  there is no closed-loop iq target.
- **Avoid when:** doing anything that matters for torque accuracy.

### `SET_CURRENT`

The workhorse. Engages `CONTROL_MODE_CURRENT` and tracks a **signed**
q-current.

- **Parameter:** current in amps, signed.
  - positive → drive torque in the "forward" direction,
  - negative → regen / reverse torque.
- **Optional parameter:** `off_delay` in seconds. If present, the bridge
  stays RUNNING at iq=0 for this long before the release path can fire.
  This is the "keep the inverter alive through brief zero-crossings"
  knob — and the single most important parameter for the freewheel case
  (see §7 of [fsm.md](fsm.md)).
- **Use when:** you have a torque (or power, via `iq = P/(1.5·ω_e·λ)`)
  target. This is the correct setter for drive, regen, and hybrid-ICE
  freewheel.
- **Gotcha:** a negative current does **not** mean "brake". Once the
  rotor crosses zero speed it will happily accelerate backwards. If you
  want "slow down and stay stopped", this is the wrong message.

### `SET_CURRENT_BRAKE`

Engages `CONTROL_MODE_CURRENT_BRAKE`. The firmware re-derives the sign of
iq every tick as `−sign(ω)·|iq|`, so the torque **always opposes motion**.

- **Parameter:** current magnitude in amps. Sign is ignored.
- **Use when:** you want "decelerate the shaft, whichever way it is
  spinning, and don't reverse it".
- **Gotcha at ω ≈ 0:** commanded iq collapses to zero. This is a
  *velocity regulator*, not a *position hold*. A slow external torque
  will drift the rotor. If you need the shaft to stay put, follow up
  with `SET_CURRENT_HANDBRAKE`.
- **Do not use for freewheel.** The sign rule produces engine-braking the
  instant the shaft moves.

### `SET_RPM`

Engages `CONTROL_MODE_SPEED`. Outer speed PID → inner current loop.

- **Parameter:** electrical RPM, signed. Divide mechanical RPM by the
  pole-pair count before sending.
- **Use when:** you actually want a regulated speed. The PID gains come
  from firmware config.

### `SET_POS`

Engages `CONTROL_MODE_POS`. Position PID.

- **Parameter:** target angle in degrees, 0..360.
- **Use when:** you want closed-loop positioning. The rotor will be
  actively held at the target after it arrives.

### `SET_CURRENT_HANDBRAKE`

Engages `CONTROL_MODE_HANDBRAKE`. Injects an open-loop static current
vector at a fixed electrical angle → restoring torque as a function of
rotor **angle**.

- **Parameter:** current magnitude in amps.
- **Use when:** you need the shaft to stay stationary. This is the only
  stock mode that genuinely clamps position.
- **Gotcha:** it does not auto-engage. You must command it yourself once
  the rotor has slowed (typical pattern: `SET_CURRENT_BRAKE` until
  `|ω| < threshold`, then switch to `SET_CURRENT_HANDBRAKE`).

### `SET_CURRENT_REL` / `SET_CURRENT_BRAKE_REL` / `SET_CURRENT_HANDBRAKE_REL`

Identical behaviour to their absolute counterparts, but the parameter is a
fraction (−1.0 … +1.0 for drive, 0.0 … 1.0 for brake/handbrake) of the
configured max motor current.

- **Use when:** you want the firmware's configured current limits to be
  the single source of truth — useful when multiple nodes with different
  motor hardware share the same commander.
- `SET_CURRENT_REL` also accepts an optional `off_delay`, same semantics
  as `SET_CURRENT`.

## 2. Picking the right message

| Goal                                                  | Message                               | Notes |
|-------------------------------------------------------|---------------------------------------|-------|
| Drive with a torque target                            | `SET_CURRENT` (positive)              | Signed; regen uses negative. |
| Regen while still moving forward                      | `SET_CURRENT` (negative)              | Watch the ω=0 reversal. |
| Decelerate, don't reverse                             | `SET_CURRENT_BRAKE`                   | Zero torque at standstill. |
| Hold the rotor still                                  | `SET_CURRENT_HANDBRAKE`               | Only mode that clamps angle. |
| Regulate a speed                                      | `SET_RPM`                             | Electrical RPM. |
| Go to an angle and stay                               | `SET_POS`                             | Degrees, 0..360. |
| Hybrid ICE, zero shaft load at any speed              | `SET_CURRENT` with `iq=0`, `off_delay≥0.5 s` @ ≥ 2 Hz | See §7 of [fsm.md](fsm.md). |
| Full coast, inverter OFF                              | stop sending, or `SET_CURRENT(0)` with no `off_delay` | Bridge falls OFF on next tick or timeout. |
| Open-loop duty                                        | `SET_DUTY`                            | Bench / characterisation only. |

## 3. Recipe book — "I want X, send this"

Each recipe lists the message name, its parameters, and the refresh rate.
Repeat the message at the stated cadence for as long as the behaviour is
desired; stop (or send the release recipe) to end it.

### 3.1 Drive with 12.5 A of q-current

- `SET_CURRENT`, current = `+12.5 A`
- @ 10 Hz
- Result: `RUNNING + CURRENT`, forward torque.

### 3.2 Regen with 8 A while moving forward

- `SET_CURRENT`, current = `−8.0 A`
- @ 10 Hz
- Result: `RUNNING + CURRENT`, opposing torque **and** charging the bus.
- Caveat: once ω crosses zero, the motor will accelerate backward. Switch
  to `SET_CURRENT_BRAKE` if you want "decelerate and stop".

### 3.2b Regen to a stop, without reversing past ω=0

You want recipe 3.2's energy recovery, but you also want the motor to
**stop and stay stopped** instead of spinning up backwards once ω crosses
zero. Two options, pick by how strict the "stay stopped" requirement is.

**Simple — `SET_CURRENT_BRAKE` is already a regenerative brake.**

- `SET_CURRENT_BRAKE`, current = `8 A` (the magnitude you wanted to regen
  at)
- @ 10 Hz
- The firmware forces iq to oppose motion, which *is* regen while the
  rotor is spinning (charges the bus via the same mechanism as negative
  `SET_CURRENT`). At ω=0 the commanded current collapses to zero, so the
  rotor will not be driven backward — but it is also **not held**. Good
  enough if small external torques don't matter.

**Strict — brake, then clamp.**

- While `|ω|` is large: `SET_CURRENT_BRAKE`, current = `8 A`, @ 10 Hz.
- Once `|ω|` drops below a small threshold (your node decides — e.g.
  a few electrical RPM): switch to `SET_CURRENT_HANDBRAKE`, current =
  `5 A`, @ 10 Hz.
- Result: regen all the way down, then a genuine position clamp. This is
  the pattern to use when the shaft must not drift afterwards.

Do **not** try to achieve this with negative `SET_CURRENT` alone — it
will reverse the rotor the moment ω hits zero (see recipe 3.2).

### 3.3 Brake at 10 A regardless of direction

- `SET_CURRENT_BRAKE`, current = `10 A`
- @ 10 Hz
- Result: `RUNNING + CURRENT_BRAKE`, torque always opposes motion; zero at
  ω=0 (does **not** hold position).

### 3.4 Hold the rotor still (position clamp)

- `SET_CURRENT_HANDBRAKE`, current = `5 A` (tune to your load)
- @ 10 Hz
- Result: `RUNNING + HANDBRAKE`, angle-dependent restoring torque.
- Typical pattern: brake first (recipe 3.3) until `|ω|` is small, then
  switch to this recipe.

### 3.5 Freewheel — hybrid ICE, zero shaft load at any speed

- `SET_CURRENT`, current = `0 A`, `off_delay = 0.5 s`
- @ ≥ 2 Hz
- Result: `RUNNING + CURRENT` with iq=id=0; FOC cancels back-EMF, no
  body-diode rectification. See §7 of [fsm.md](fsm.md).
- Transition to drive/regen is one tick away — just change `current`.

### 3.6 Full coast, inverter OFF

- `SET_CURRENT`, current = `0 A`, no `off_delay` (or `off_delay = 0`)
- once, then stop transmitting
- Result: bridge releases to `OFF + NONE` on the next tick. Phases
  high-Z. (Equivalently: stop sending anything and let the watchdog
  timeout do it.)
- Caveat: at high speed the body diodes may rectify into the bus. Prefer
  recipe 3.5 on a hybrid driveline.

### 3.7 Regulate speed at 3000 electrical RPM

- `SET_RPM`, rpm = `3000` (electrical; divide mechanical RPM by pole
  pairs)
- @ 10 Hz
- Result: `RUNNING + SPEED`, outer PID drives current loop.

### 3.8 Go to angle 90° and hold

- `SET_POS`, pos = `90.0` degrees
- @ 10 Hz
- Result: `RUNNING + POS`, position PID. No follow-up handbrake needed —
  the PID keeps holding at the target.

### 3.9 Relative drive at 50 % of configured max current

- `SET_CURRENT_REL`, current_rel = `+0.5`
- @ 10 Hz
- Result: same as recipe 3.1 but the absolute amps come from firmware
  config. Useful when one commander drives several different VESCs.

## 4. Mode transitions in practice

Because each message is its own mode selector, a command sequence is a
mode sequence:

```
  SET_CURRENT(10)            → CURRENT,   drive 10 A
  SET_CURRENT_BRAKE(15)      → CURRENT_BRAKE, decel with 15 A opposing ω
  SET_CURRENT_HANDBRAKE(5)   → HANDBRAKE, clamp angle with 5 A
  SET_CURRENT(0, off_delay=0.5) → CURRENT, iq=0 (freewheel)
  (stop transmitting)        → timeout → OFF
```

No explicit "release before switching" step is needed. The one thing the
firmware will not do for you is the `BRAKE → HANDBRAKE` handoff at low
speed — that transition is your node's responsibility, because only your
node knows how fast is "slow enough".

## 5. Not-a-message reminders

- There is no "enable" or "disable" message.
- There is no "set mode" message distinct from the setters.
- There is no acknowledgement for setpoint messages. Telemetry arrives on
  separate `CAN_STATUS_*` IDs (out of scope here).
- There is no merge/blend between overlapping setters. If two controllers
  on the bus send conflicting setpoints to the same `controller_id`, the
  motor follows whichever message arrived most recently.
