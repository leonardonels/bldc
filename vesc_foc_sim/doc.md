# VESC FOC Simulator — File-by-File Documentation

This document describes every file in `vesc_foc_sim/`, explains what it does, and
shows exactly how each file reaches into the main BLDC firmware repository to borrow
real production code instead of reimplementing it.

---

## Repository layout

```
vesc_foc_sim/
├── CMakeLists.txt                  build: compiles firmware C lib + 3 ROS2 nodes
├── package.xml                     ROS2 package manifest
├── doc.md                          ← this file
├── README.md                       quick-start guide and architecture overview
├── config/
│   └── motor_params.yaml           runtime parameters for all three nodes
├── launch/
│   └── sim.launch.py               launches all three nodes with motor_params.yaml
├── src/
│   ├── foc_bridge.c                C-only bridge to BLDC firmware (compiled as C)
│   ├── pmsm_plant_node.cpp         PMSM motor model (ROS2 node)
│   ├── vesc_controller_node.cpp    FOC current controller (ROS2 node)
│   └── ecu_sim_node.cpp            ECU / setpoint generator (ROS2 node)
└── include/vesc_foc_sim/
    └── adapter.hpp                 C++ adapter exposing foc_math:: namespace
```

The BLDC firmware sources used by the sim live in the **parent repository**:

```
../motor/foc_math.c          real FOC observer and PLL (compiled as C into the sim)
../motor/foc_math.h          observer_state, foc_observer_update, foc_pll_run
../util/utils_math.c         utils_map, utils_min_abs, utils_fast_sincos_better, …
../util/utils_math.h         all math utility declarations
../datatypes.h               mc_configuration, motor_all_state_t, observer_state
../tests/utils_math/ch.h     ChibiOS stub (systime_t, thread_t) — upstream test infra
```

---

## `CMakeLists.txt` — build wiring

**Purpose:** glues the real BLDC firmware C sources into the ROS2 package as a
static library, then builds the three executable nodes.

### Firmware static library — `vesc_firmware_c`

```cmake
add_library(vesc_firmware_c STATIC
    ${BLDC_ROOT}/motor/foc_math.c       # ← real BLDC firmware
    ${BLDC_ROOT}/util/utils_math.c      # ← real BLDC firmware
    src/foc_bridge.c                    # ← sim bridge (C only)
)
```

`BLDC_ROOT` is set to `${CMAKE_CURRENT_SOURCE_DIR}/..`, i.e., the root of the
cloned BLDC repository.  All three C translation units are compiled with the
**C compiler** (enforced via `LINKER_LANGUAGE C`) so that `datatypes.h`, which
uses C99 designated initialisers and `_Bool`, is never processed by the C++
compiler.

The include path given to `vesc_firmware_c` is:

| Path | Why it is needed |
|------|-----------------|
| `../tests/utils_math` | provides `ch.h` — a minimal ChibiOS stub (`systime_t`, `thread_t`) so `datatypes.h` compiles on Linux without the RTOS |
| `../` (BLDC root) | `#include "datatypes.h"` resolves here |
| `../motor` | `#include "foc_math.h"` resolves here |
| `../util` | `#include "utils_math.h"` resolves here |

### Sim executables

Each node executable is linked against `vesc_firmware_c`, which brings the real
firmware symbols in:

```cmake
target_link_libraries(pmsm_plant      vesc_firmware_c)
target_link_libraries(vesc_controller vesc_firmware_c)
# ecu_sim does NOT link vesc_firmware_c — it has no firmware dependency
```

---

## `src/foc_bridge.c` — C bridge to BLDC firmware

**Purpose:** the only file in the sim that `#include`s firmware headers
(`foc_math.h`, `utils_math.h`, `datatypes.h`).  It constructs the minimal
`mc_configuration` and `motor_all_state_t` structs required by the firmware
API and wraps each firmware call with a **plain scalar interface** so the C++
nodes never have to see the C-only types.

### Firmware functions called directly

| Bridge function | BLDC firmware call | Source file |
|---|---|---|
| `foc_bridge_observer_update()` | `foc_observer_update(v_alpha, v_beta, i_alpha, i_beta, dt, &state, &phase, &motor)` | `motor/foc_math.c` |
| `foc_bridge_pll_run()` | `foc_pll_run(phase_input, dt, phase_var, speed_var, &conf)` | `motor/foc_math.c` |
| `vm_step_bridge()` | `utils_norm_angle_rad()`, `utils_fast_sincos_better()` | `util/utils_math.c` |
| `compute_limits_bridge()` | `utils_map()`, `utils_min_abs()` | `util/utils_math.c` |

### `foc_bridge_observer_update` — wraps `foc_observer_update`

```c
// motor/foc_math.h signature:
void foc_observer_update(float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float dt,
                         observer_state *state, float *phase,
                         motor_all_state_t *motor);
```

The bridge builds a throw-away `mc_configuration` with just the fields the
MXLEMMING observer needs (`foc_motor_r`, `foc_motor_l`, `foc_motor_flux_linkage`,
`foc_observer_type = FOC_OBSERVER_MXLEMMING`, `foc_sat_comp_mode = SAT_COMP_DISABLED`)
and a throw-away `motor_all_state_t` pointing at that config.  The
`observer_state` fields (`x1`, `x2`, `lambda_est`, `i_alpha_last`, `i_beta_last`)
are passed in and out as individual `float*` pointers so the caller never needs
to `#include "foc_math.h"`.

### `foc_bridge_pll_run` — wraps `foc_pll_run`

```c
// motor/foc_math.h signature:
void foc_pll_run(float phase, float dt,
                 float *phase_var, float *speed_var,
                 const mc_configuration *conf);
```

The bridge builds a minimal `mc_configuration` with only `foc_pll_kp` and
`foc_pll_ki` set, then calls the real firmware PLL.

### `vm_step_bridge` — motor electrical ODE

Ported from `motor/virtual_motor.c` (`run_virtual_motor_electrical`,
`run_virtual_motor_mechanics`, `run_virtual_motor_park_clark_inverse`).
After integrating the dq ODE, it calls two utility functions from the BLDC
firmware:

```c
utils_norm_angle_rad(&theta_e);            // util/utils_math.c
utils_fast_sincos_better(*theta_e, &s, &c); // util/utils_math.c
```

Deviations from the original `virtual_motor.c`:
- Input is `vd`/`vq` (already in dq frame) — no input Park transform needed.
- Electrical angle is tracked as `theta_e += p * omega_m * dt` (true electrical),
  fixing a latent multi-pole bug where the original accumulated a mechanical angle.
- Reluctance torque is included: `τe = (3/2)·p·(λ + (Ld − Lq)·id)·iq`.

### `compute_limits_bridge` — override limits

Ported from `motor/mc_interface.c :: update_override_limits()`.  All the
waterfall logic is re-implemented here in C, calling:

```c
utils_map(val, in_min, in_max, out_min, out_max);  // util/utils_math.c
utils_min_abs(a, b);                                // util/utils_math.c
```

Temperature derating, RPM derating, battery cutoff, wattage limits, and
input-current-to-phase-current mapping are all handled here.

---

## `include/vesc_foc_sim/adapter.hpp` — C++ adapter

**Purpose:** the only header the C++ nodes include for firmware-backed math.
Declares the `extern "C"` bridge function prototypes (so C++ can call into the
C bridge) and wraps them in the `foc_math::` namespace with clean, typed C++
APIs.

### Inline math (no bridge call)

These are trivially short and are kept as `inline` C++ to avoid call overhead:

| `foc_math::` function | Mirrors in BLDC firmware |
|---|---|
| `clarke(ia, ib, i_alpha, i_beta)` | Clarke transform inlined in `motor/mcpwm_foc.c` |
| `park(i_alpha, i_beta, cos, sin, id, iq)` | Park transform inlined in `motor/mcpwm_foc.c` |
| `park_inv(vd, vq, cos, sin, v_alpha, v_beta)` | Inverse Park inlined in `motor/mcpwm_foc.c` |
| `lp_fast(val, sample, k)` | `UTILS_LP_FAST` macro from `util/utils_math.h` |
| `norm_angle_rad(a)` | `utils_norm_angle_rad` static inline in `util/utils_math.h` |
| `truncate_abs(x, max)` | `utils_truncate_number_abs` from `util/utils_math.h` |
| `sq(x)` | used inline throughout `motor/mcpwm_foc.c` |

### Firmware-backed functions (via bridge)

| `foc_math::` function | Calls | Firmware origin |
|---|---|---|
| `observer_update(…, ObserverState&)` | `foc_bridge_observer_update` → `foc_observer_update` | `motor/foc_math.c` |
| `pll_run(…, PllState&)` | `foc_bridge_pll_run` → `foc_pll_run` | `motor/foc_math.c` |
| `vm_step(…)` | `vm_step_bridge` → `utils_norm_angle_rad`, `utils_fast_sincos_better` | `util/utils_math.c` |
| `compute_override_limits(params)` | `compute_limits_bridge` → `utils_map`, `utils_min_abs` | `util/utils_math.c` |

### Field-weakening (`fw_compute`)

```
fw_compute(duty_abs, fw_duty_start, l_max_duty, fw_current_max,
           fw_i_set_prev, fw_ramp_time, dt, cc_min_current)
```

A direct extraction of the duty-ramp math from `motor/foc_math.c ::
foc_run_fw()` for `CONTROL_MODE_CURRENT`.  Kept as inline C++ — it references
no firmware types, only scalar arithmetic.

### State structs

| C++ struct | Mirrors firmware struct | Defined in |
|---|---|---|
| `ObserverState` | `observer_state` | `motor/foc_math.h` |
| `PllState` | `phase_var` / `speed_var` in `foc_pll_run` | `motor/foc_math.h` |
| `OverrideLimitParams` (alias) | `limits_params_t` in `foc_bridge.c` | `src/foc_bridge.c` |
| `OverrideLimitResult` (alias) | `limits_result_t` in `foc_bridge.c` | `src/foc_bridge.c` |

---

## `src/pmsm_plant_node.cpp` — PMSM plant

**Purpose:** simulates the physical PMSM motor.  Runs a fixed-rate timer at
`sim_dt_s` and integrates the dq-frame ODE on every tick.

### Firmware dependency

The node includes only `adapter.hpp` and calls a single firmware-backed
function:

```cpp
foc_math::vm_step(
    id_int_, id_, iq_, omega_m_, theta_e_,   // plant state (in/out)
    vd_, vq_, dt_,                            // inputs
    R_, Ld_, Lq_, lambda_, J_, tau_load_, p_,// motor params
    i_alpha, i_beta, torque);                 // outputs
```

Call chain:

```
pmsm_plant_node.cpp
  └─ foc_math::vm_step()                      [adapter.hpp — inline]
       └─ vm_step_bridge()                    [foc_bridge.c — C]
            ├─ utils_norm_angle_rad()         [util/utils_math.c — BLDC firmware]
            └─ utils_fast_sincos_better()     [util/utils_math.c — BLDC firmware]
```

### ODE integrated by `vm_step_bridge`

```
id_int  += (Vd + ωe·Lq·iq  − R·id) · dt / Ld
id       = id_int − λ/Ld
diq/dt   = (Vq − ωe·(Ld·id + λ) − R·iq) / Lq
τe       = (3/2)·p·(λ + (Ld−Lq)·id)·iq
dωm/dt   = (τe − τload) / J
dθe/dt   = p · ωm
```

### Topics

| Direction | Topic | Payload |
|---|---|---|
| Subscribed | `/foc/vd` | d-axis voltage from controller [V] |
| Subscribed | `/foc/vq` | q-axis voltage from controller [V] |
| Published | `/foc/id` | d-axis current [A] |
| Published | `/foc/iq` | q-axis current [A] |
| Published | `/foc/i_alpha` | α-frame current [A] |
| Published | `/foc/i_beta` | β-frame current [A] |
| Published | `/foc/theta_real` | true electrical angle [rad] |
| Published | `/foc/omega` | true electrical speed [rad/s] |
| Published | `/foc/torque` | electromagnetic torque [Nm] |

---

## `src/vesc_controller_node.cpp` — VESC FOC controller

**Purpose:** implements the FOC current controller, ported from
`motor/mcpwm_foc.c :: control_current()`.  Runs at `sim_dt_s` and produces the
dq voltage commands the plant needs.

### Firmware dependencies

The node includes `adapter.hpp` and uses every firmware-backed path in it:

```cpp
// 1. Observer (Phase 2 only)
const float theta_obs = foc_math::observer_update(
    vd_alpha_last_, vq_beta_last_,
    i_alpha_, i_beta_,
    dt_, R_, L_, lambda_, obs_state_);

// 2. PLL (Phase 2 only)
foc_math::pll_run(theta_obs, dt_, pll_kp_, pll_ki_, pll_);

// 3. Park transform (always)
foc_math::park(i_alpha_, i_beta_, cos_t, sin_t, id_meas, iq_meas);

// 4. Inverse Park (Phase 2 only — store v_alpha/v_beta for next observer call)
foc_math::park_inv(vd, vq, cos_t, sin_t, vd_alpha_last_, vq_beta_last_);

// 5. Low-pass filter (always)
foc_math::lp_fast(id_filt_, id_meas, kf_);

// 6. Field weakening (always)
fw_i_set_ = foc_math::fw_compute(duty_abs, fw_duty_start_, max_duty_,
                                  fw_i_max_, fw_i_set_, fw_ramp_, dt_, cc_min_i_);

// 7. Override limits (always)
const foc_math::OverrideLimitResult lim = foc_math::compute_override_limits({…});
```

### Full call chain to BLDC firmware

```
vesc_controller_node.cpp
  ├─ foc_math::observer_update()              [adapter.hpp — inline]
  │    └─ foc_bridge_observer_update()        [foc_bridge.c — C]
  │         └─ foc_observer_update()          [motor/foc_math.c — BLDC firmware]
  ├─ foc_math::pll_run()                      [adapter.hpp — inline]
  │    └─ foc_bridge_pll_run()                [foc_bridge.c — C]
  │         └─ foc_pll_run()                  [motor/foc_math.c — BLDC firmware]
  ├─ foc_math::park()                         [adapter.hpp — inline, no bridge]
  ├─ foc_math::park_inv()                     [adapter.hpp — inline, no bridge]
  ├─ foc_math::lp_fast()                      [adapter.hpp — inline, mirrors UTILS_LP_FAST]
  ├─ foc_math::fw_compute()                   [adapter.hpp — inline, extracted from foc_run_fw]
  └─ foc_math::compute_override_limits()      [adapter.hpp — inline]
       └─ compute_limits_bridge()             [foc_bridge.c — C]
            ├─ utils_map()                    [util/utils_math.c — BLDC firmware]
            └─ utils_min_abs()               [util/utils_math.c — BLDC firmware]
```

### Control loop steps (per tick)

1. **Watchdog** — if no `/iq_ref` received within `timeout_s`, force `iq_ref = 0`.
2. **Bus voltage guard** — inhibit if `vbus` outside `[l_min_vin, l_max_vin]`.
3. **Speed limit** — zero `iq_ref` if `|erpm| > l_max_erpm`.
4. **Override limits** — compute thermally/electrically derated current limits
   via `compute_override_limits` (ported from `mc_interface.c`).
5. **Angle selection** — Phase 1: use `/foc/theta_real`; Phase 2: run MXLEMMING
   observer (`foc_observer_update`) + PLL (`foc_pll_run`).
6. **Park transform** — αβ → dq via `foc_math::park`.
7. **Low-pass filter** — smooth `id`/`iq` for decoupling feedforward.
8. **Field weakening** — compute `id_ref` from duty ratio via `fw_compute`.
9. **PI controllers** — `vd/vq` from proportional + integral terms.
10. **Cross-coupling + back-EMF decoupling** — `FOC_CC_DECOUPLING_CROSS_BEMF`
    from `motor/mcpwm_foc.c`.
11. **Voltage saturation + anti-windup** — clamp `vd`, `vq` to the modulation
    hexagon.

### Topics

| Direction | Topic | Payload |
|---|---|---|
| Subscribed | `/iq_ref` | current setpoint [A] (also resets watchdog) |
| Subscribed | `/foc/i_alpha` | α-frame current [A] |
| Subscribed | `/foc/i_beta` | β-frame current [A] |
| Subscribed | `/foc/theta_real` | true electrical angle [rad] (Phase 1) |
| Subscribed | `/foc/omega` | true electrical speed [rad/s] |
| Subscribed | `/foc/vbus` | bus voltage [V] |
| Published | `/foc/vd` | d-axis voltage command [V] |
| Published | `/foc/vq` | q-axis voltage command [V] |
| Published | `/foc/theta_est` | observer angle estimate [rad] |
| Published | `/foc/pi_d_err` | PI d-axis error [A] (debug) |
| Published | `/foc/pi_q_err` | PI q-axis error [A] (debug) |
| Published | `/foc/id_ref` | effective Id reference [A] |
| Published | `/foc/iq_ref_clamped` | Iq after limits [A] |
| Published | `/foc/lo_current_max` | derated current limit [A] |
| Published | `/foc/fault` | 1.0 if over-temperature fault active |

---

## `src/ecu_sim_node.cpp` — ECU simulator

**Purpose:** simulates the vehicle ECU.  Publishes a current setpoint (`/iq_ref`)
and bus voltage (`/foc/vbus`) at `publish_rate_hz`.  Has no direct BLDC firmware
dependency — it is pure C++/ROS2.

Three selectable profiles are available via the `profile` parameter:

### `sine`

```
iq_ref = amp · sin(2π · freq_hz · (t − t_start_s))    for t ≥ t_start_s
```

### `sine_coast`

Alternates between a sine window (`sine_duration_s`) and a silent window
(`coast_duration_s`).  During the silent window `/iq_ref` is not published at
all, which triggers the controller watchdog after `timeout_s`.

```
phase = (t − t_start_s) mod (sine_duration_s + coast_duration_s)
if phase < sine_duration_s:  publish amp · sin(2π · freq_hz · phase)
else:                         silence  →  watchdog fires
```

### `speed_profile`

Closed-loop outer PI speed loop driven by YAML waypoints.  Subscribes to
`/foc/omega` and outputs `iq_ref`:

```
error    = omega_target(t) − omega_measured
integral += ki_speed · error · dt   (with back-calculation anti-windup)
iq_ref   = clamp(kp_speed · error + integral, −l_current_max, l_current_max)
```

Waypoints use step-hold interpolation: the target holds its value until the next
`t_s` entry.

### Topics

| Direction | Topic | Payload |
|---|---|---|
| Published | `/iq_ref` | current command [A] |
| Published | `/foc/vbus` | bus voltage [V] |
| Subscribed | `/foc/omega` | measured speed [rad/s] (`speed_profile` only) |

---

## `config/motor_params.yaml` — configuration

**Purpose:** single YAML file loaded by all three nodes at launch.  Parameters
are placed under the `/**` namespace so every node receives every key regardless
of its ROS2 node name.

Each parameter that comes from a VESC Tool XML export carries a comment showing
the XML field name (e.g. `# foc_motor_r`) so values can be cross-referenced
directly against a hardware profile.

Key parameter groups:

| Group | Parameters | Origin |
|---|---|---|
| Motor electrical | `R`, `Ld`, `Lq`, `lambda`, `p` | VESC Tool XML `foc_motor_*` |
| Motor mechanical | `J_tot`, `tau_load` | estimated / chassis team |
| FOC controller | `current_kp`, `current_ki`, `pll_kp`, `pll_ki` | XML `foc_current_*`, `foc_pll_*` |
| Field weakening | `fw_current_max`, `fw_duty_start`, `fw_ramp_time` | XML `foc_fw_*` |
| Current limits | `l_current_max`, `l_current_min`, `l_in_current_max`, … | XML `l_*` |
| Temperature limits | `t_fet`, `t_motor`, `l_temp_fet_*`, `l_temp_motor_*` | XML `l_temp_*` |
| Battery / power | `l_battery_cut_*`, `l_watt_max`, `l_watt_min` | XML |
| Simulation | `sim_dt_s`, `vbus`, `timeout_s`, `use_observer` | sim-specific |
| ECU profile | `profile`, `freq_hz`, `iq_ref_amplitude`, … | sim-specific |
| Speed waypoints | `speed_waypoints.t_s`, `speed_waypoints.omega_target_radps` | sim-specific |

---

## `launch/sim.launch.py` — launch file

**Purpose:** starts all three nodes in one command with a shared parameter file.

```bash
ros2 launch vesc_foc_sim sim.launch.py
# Override parameter file:
ros2 launch vesc_foc_sim sim.launch.py params:=/path/to/custom_params.yaml
```

The launch file:
1. Resolves the installed `motor_params.yaml` via `get_package_share_directory`.
2. Declares a `params` launch argument so users can override at runtime.
3. Spawns `pmsm_plant`, `vesc_controller`, and `ecu_sim` with the same YAML
   file so all three share a consistent parameter set.

---

## End-to-end data flow

```
ecu_sim_node.cpp
  │  publishes /iq_ref, /foc/vbus
  │
  ▼
vesc_controller_node.cpp  ◄── /foc/i_alpha, /foc/i_beta, /foc/theta_real,
  │                              /foc/omega  (from pmsm_plant)
  │  calls (via adapter.hpp → foc_bridge.c):
  │    foc_observer_update()   motor/foc_math.c
  │    foc_pll_run()           motor/foc_math.c
  │    utils_map()             util/utils_math.c
  │    utils_min_abs()        util/utils_math.c
  │
  │  publishes /foc/vd, /foc/vq
  │
  ▼
pmsm_plant_node.cpp
  │  calls (via adapter.hpp → foc_bridge.c):
  │    utils_norm_angle_rad()          util/utils_math.c
  │    utils_fast_sincos_better()      util/utils_math.c
  │
  │  publishes /foc/id, /foc/iq, /foc/i_alpha, /foc/i_beta,
  │            /foc/theta_real, /foc/omega, /foc/torque
  │
  └──────────────────────────────────────────────────► (feedback to controller)
```

---

## How the BLDC firmware is used without modification

The sim compiles `motor/foc_math.c` and `util/utils_math.c` **verbatim** —
no patches, no preprocessor tricks beyond the ChibiOS stub.  When the firmware
is updated (e.g. observer algorithm improvement, new limit logic), the sim
picks up the change automatically on the next `colcon build`.  Only
`foc_bridge.c` needs updating if the firmware API changes.

The isolation boundary is:

```
C++ world                C world
────────────────────     ────────────────────────────────────────────────
adapter.hpp        ──►   foc_bridge.c
  foc_math:: ns           ├── #include "foc_math.h"   (motor/foc_math.h)
                          ├── #include "utils_math.h" (util/utils_math.h)
                          └── #include "datatypes.h"  (root datatypes.h)
                                └── #include "ch.h"   (tests/utils_math/ch.h stub)
```

No C++ file ever sees `datatypes.h`, `foc_math.h`, or `utils_math.h` directly.
