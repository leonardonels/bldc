# VESC FOC Simulator

A ROS2 closed-loop simulation of the VESC FOC current controller running against
a PMSM plant model. The goal is to test and validate the real firmware math
(`motor/foc_math.c`) on a Linux host before deploying to hardware.

The sim is structured as three independent ROS2 nodes that communicate over topics,
mirroring how the real ECU, VESC, and motor interact on the vehicle.

```
┌─────────────┐  /iq_ref   ┌──────────────────┐  /foc/vd, /foc/vq  ┌──────────────┐
│   ecu_sim   │ ─────────► │ vesc_controller  │ ──────────────────► │ pmsm_plant   │
│             │  /foc/vbus │                  │                     │              │
└─────────────┘            └──────────────────┘                     └──────┬───────┘
       ▲                           ▲                                        │
       │      /foc/omega           │  /foc/i_alpha, /foc/i_beta             │
       └───────────────────────────┴────────────────────────────────────────┘
                                      /foc/theta_real, /foc/omega
```

---

## Architecture

### Firmware integration

Rather than re-implementing the FOC math, the sim compiles the actual firmware
sources directly. When the firmware is updated, the sim picks up the changes on
the next build automatically — no manual porting needed.

```
motor/foc_math.c          ← real VESC firmware (compiled as C)
util/utils_math.c         ← real VESC firmware (compiled as C)
src/foc_bridge.c          ← C-only bridge: wraps motor_all_state_t with a plain
                             scalar interface so C++ nodes never see datatypes.h
include/vesc_foc_sim/
  adapter.hpp             ← C++ adapter: exposes foc_math:: namespace to the nodes,
                             calls the real firmware via foc_bridge
tests/utils_math/ch.h     ← ChibiOS stub (upstream test infrastructure):
                             provides systime_t/thread_t so datatypes.h compiles
                             on Linux without the RTOS
```

The C++ nodes include only `adapter.hpp`. The firmware headers (`datatypes.h`,
`foc_math.h`, `utils_math.h`) are seen exclusively by the C compiler, which
avoids C vs C++ incompatibilities in `datatypes.h`.

---

## Nodes

### `pmsm_plant` — `src/pmsm_plant_node.cpp`

Physical model of the PMSM motor. Integrates the dq-frame ODE with a 4th-order
Runge-Kutta stepper and publishes the resulting electrical state.

**ODE (rotor reference frame, SPM):**
```
dId/dt  = (Vd − R·Id + ωe·Lq·Iq) / Ld
dIq/dt  = (Vq − R·Iq − ωe·Ld·Id − ωe·λ) / Lq
dωm/dt  = (τe − τload − B·ωm) / J
dθe/dt  = p · ωm
```

| Published topic   | Type    | Description                         |
|-------------------|---------|-------------------------------------|
| `/foc/id`         | Float64 | d-axis current [A]                  |
| `/foc/iq`         | Float64 | q-axis current [A]                  |
| `/foc/i_alpha`    | Float64 | α-frame current [A]                 |
| `/foc/i_beta`     | Float64 | β-frame current [A]                 |
| `/foc/theta_real` | Float64 | true electrical angle [rad]         |
| `/foc/omega`      | Float64 | true electrical speed [rad/s]       |
| `/foc/torque`     | Float64 | electromagnetic torque [Nm]         |

---

### `vesc_controller` — `src/vesc_controller_node.cpp`

FOC current controller, ported from `motor/mcpwm_foc.c` (`control_current()`).
Runs the real firmware observer and PLL via `adapter.hpp` → `foc_bridge.c` →
`motor/foc_math.c`.

**Control loop (runs at `sim_dt_s`):**
1. Watchdog — coasts if no `/iq_ref` received within `timeout_s`
2. Bus voltage and speed limit checks
3. Angle selection — Phase 1: perfect angle from plant; Phase 2: MXLEMMING
   observer + PLL (`use_observer: true`)
4. Park transform (αβ → dq)
5. Field weakening — computes Id reference from duty ratio
6. PI current controllers for Id and Iq
7. Cross-coupling + back-EMF decoupling (`FOC_CC_DECOUPLING_CROSS_BEMF`)
8. Voltage saturation with anti-windup
9. Publishes Vd, Vq

| Subscribed topic  | Description                              |
|-------------------|------------------------------------------|
| `/iq_ref`         | current setpoint [A] — also resets watchdog |
| `/foc/i_alpha`    | α-frame current from plant [A]          |
| `/foc/i_beta`     | β-frame current from plant [A]          |
| `/foc/theta_real` | true electrical angle [rad] (Phase 1)   |
| `/foc/omega`      | true electrical speed [rad/s]           |
| `/foc/vbus`       | bus voltage [V]                         |

| Published topic        | Description                        |
|------------------------|------------------------------------|
| `/foc/vd`              | d-axis voltage command [V]         |
| `/foc/vq`              | q-axis voltage command [V]         |
| `/foc/theta_est`       | observer angle estimate [rad]      |
| `/foc/pi_d_err`        | PI d-axis error [A] (debug)        |
| `/foc/pi_q_err`        | PI q-axis error [A] (debug)        |
| `/foc/id_ref`          | effective Id reference [A]         |
| `/foc/iq_ref_clamped`  | Iq after limits [A]                |

---

### `ecu_sim` — `src/ecu_sim_node.cpp`

Simulates the vehicle ECU. Publishes the current setpoint and bus voltage.
Three profiles are available, selected via the `profile` parameter.

**`sine`** — open-loop sinusoidal current reference:
```
iq_ref = amp · sin(2π · freq_hz · (t − t_start_s))    for t ≥ t_start_s
```

**`sine_coast`** — repeating sine + watchdog test. Runs `sine_duration_s` of sine, then
stops publishing `/iq_ref` entirely for `coast_duration_s`, then repeats. The silence
triggers the controller watchdog after `timeout_s`, forcing `iq_ref = 0` and letting the
motor coast. `/foc/vbus` is always published regardless.
```
phase = (t − t_start_s) mod (sine_duration_s + coast_duration_s)
if phase < sine_duration_s:  publish amp · sin(2π · freq_hz · phase)
else:                         publish nothing  →  watchdog fires after timeout_s
```

**`speed_profile`** — closed-loop outer PI speed loop tracking YAML waypoints:
```
error    = omega_target(t) − omega_measured
integral += ki_speed · error · dt   (with anti-windup clamp)
iq_ref   = clamp(kp_speed · error + integral, −l_current_max, l_current_max)
```
Waypoints are step-held: the target holds its value until the next waypoint time.

---

## Files

```
vesc_foc_sim/
├── CMakeLists.txt                  build: C firmware lib + 3 ROS2 executables
├── package.xml
├── config/
│   └── motor_params.yaml           all parameters for all three nodes
├── launch/
│   └── sim.launch.py               launches all three nodes with the yaml config
├── src/
│   ├── pmsm_plant_node.cpp         PMSM plant (RK4 ODE integrator)
│   ├── vesc_controller_node.cpp    FOC current controller
│   ├── ecu_sim_node.cpp            ECU current/speed reference generator
│   └── foc_bridge.c               C bridge to firmware (compiled as C)
└── include/vesc_foc_sim/
    └── adapter.hpp                 C++ adapter exposing foc_math:: namespace
```

---

## Configuration — `config/motor_params.yaml`

All parameters are shared across the three nodes via the `/**` namespace.
Key groups:

| Group            | Parameters                                              |
|------------------|---------------------------------------------------------|
| Motor electrical | `R`, `Ld`, `Lq`, `lambda`, `p`                         |
| Motor mechanical | `J_tot`, `B`, `tau_load`                               |
| FOC controller   | `current_kp`, `current_ki`, `pll_kp`, `pll_ki`        |
| Field weakening  | `fw_current_max`, `fw_duty_start`, `fw_ramp_time`      |
| Limits           | `l_current_max`, `l_max_erpm`, `l_min_vin`, ...        |
| Simulation       | `sim_dt_s`, `vbus`, `timeout_s`, `use_observer`        |
| ECU profile      | `profile`, `freq_hz`, `iq_ref_amplitude`, `t_start_s`  |
| Speed waypoints  | `speed_waypoints.t_s`, `speed_waypoints.omega_target_radps` |

Parameters with XML source comments in the yaml map directly to VESC Tool
export fields (`foc_motor_r`, `foc_pll_kp`, etc.) for easy cross-referencing.

---

## Usage

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select vesc_foc_sim

# Run
source install/setup.bash
ros2 launch vesc_foc_sim sim.launch.py

# Override parameters at launch
ros2 launch vesc_foc_sim sim.launch.py params:=/path/to/custom_params.yaml

# Monitor topics
ros2 topic echo /foc/omega
ros2 topic echo /foc/torque
ros2 topic hz   /foc/vd
```

---

## Phases

| Phase | `use_observer` | Angle source         | Purpose                              |
|-------|----------------|----------------------|--------------------------------------|
| 1     | `false`        | `/foc/theta_real`    | Validate PI gains with perfect angle |
| 2     | `true`         | MXLEMMING observer   | Validate observer + PLL convergence  |

Start with Phase 1 to tune current PI gains, then switch to Phase 2 to test
the observer. The PLL state is kept in sync during Phase 1 so switching is
bumpless.
