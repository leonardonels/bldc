// foc_math.hpp — pure C++ port of VESC FOC math (no ChibiOS/STM32 deps)
// Ported from: motor/foc_math.c, util/utils_math.h  (vedderb/bldc)
// Observer: MXLEMMING (foc_observer_type == 1, as set in the FS XML config)
// PI decoupling: CROSS_BEMF (foc_cc_decoupling == 3)

#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace foc_math {

// ── Constants ────────────────────────────────────────────────────────────────
static constexpr float ONE_BY_SQRT3 = 0.57735026919f;
static constexpr float TWO_BY_SQRT3 = 1.15470053838f;
static constexpr float SQRT3_BY_2   = 0.86602540378f;

// ── Scalar utilities ─────────────────────────────────────────────────────────
inline float sq(float x) { return x * x; }
inline float norm2(float x, float y) { return std::sqrt(sq(x) + sq(y)); }

inline void nan_zero(float &x) { if (std::isnan(x)) x = 0.0f; }

inline void norm_angle_rad(float &a) {
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    while (a >=  static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
}

inline void truncate_abs(float &x, float max_val) {
    if      (x >  max_val) x =  max_val;
    else if (x < -max_val) x = -max_val;
}

inline void truncate(float &x, float lo, float hi) {
    if      (x > hi) x = hi;
    else if (x < lo) x = lo;
}

// First-order low-pass: val += -k*(val - sample)
inline void lp_fast(float &val, float sample, float k) {
    val -= k * (val - sample);
}

// Polynomial approximation of atan2 — identical to VESC utils_fast_atan2
inline float fast_atan2(float y, float x) {
    const float abs_y = std::fabs(y) + 1e-20f;
    float angle;
    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        angle   = ((0.1963f * r * r) - 0.9817f) * r + (static_cast<float>(M_PI) / 4.0f);
    } else {
        float r = (x + abs_y) / (abs_y - x);
        angle   = ((0.1963f * r * r) - 0.9817f) * r + (3.0f * static_cast<float>(M_PI) / 4.0f);
    }
    nan_zero(angle);
    return (y < 0.0f) ? -angle : angle;
}

// ── Clarke transform (amplitude-invariant, balanced 3-phase) ─────────────────
//   i_alpha = ia
//   i_beta  = ONE_BY_SQRT3·ia + TWO_BY_SQRT3·ib
// Matches VESC ISR: i_alpha = i_a; i_beta = ONE_BY_SQRT3*ia + TWO_BY_SQRT3*ib
inline void clarke(float ia, float ib,
                   float &i_alpha, float &i_beta) {
    i_alpha = ia;
    i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
}

// ── Park transform ────────────────────────────────────────────────────────────
//   id =  cos·iα + sin·iβ
//   iq = -sin·iα + cos·iβ    (= cos·iβ − sin·iα, same thing)
// Matches VESC control_current():
//   id = c*i_alpha + s*i_beta;  iq = c*i_beta - s*i_alpha
inline void park(float i_alpha, float i_beta,
                 float cos_theta, float sin_theta,
                 float &id, float &iq) {
    id =  cos_theta * i_alpha + sin_theta * i_beta;
    iq =  cos_theta * i_beta  - sin_theta * i_alpha;
}

// ── Inverse Park ──────────────────────────────────────────────────────────────
//   v_alpha = cos·vd − sin·vq
//   v_beta  = sin·vd + cos·vq
inline void park_inv(float vd, float vq,
                     float cos_theta, float sin_theta,
                     float &v_alpha, float &v_beta) {
    v_alpha = cos_theta * vd - sin_theta * vq;
    v_beta  = sin_theta * vd + cos_theta * vq;
}

// ── Luenberger-style observer state — MXLEMMING variant ──────────────────────
struct ObserverState {
    float x1{0.0f};
    float x2{0.0f};
    float i_alpha_last{0.0f};
    float i_beta_last{0.0f};
};

// MXLEMMING observer update — foc_observer_type == 1
// Ref: MESC FOC project (David Molony); ported from motor/foc_math.c lines 108-138
// Returns estimated electrical angle θ [rad].
inline float observer_update(
        float v_alpha, float v_beta,
        float i_alpha, float i_beta,
        float dt,
        float R, float L, float lambda,
        ObserverState &s)
{
    s.x1 += (v_alpha - R * i_alpha) * dt - L * (i_alpha - s.i_alpha_last);
    s.x2 += (v_beta  - R * i_beta)  * dt - L * (i_beta  - s.i_beta_last);

    truncate_abs(s.x1, lambda);
    truncate_abs(s.x2, lambda);

    nan_zero(s.x1);
    nan_zero(s.x2);

    // Prevent magnitude collapse (matches VESC: if mag < 0.5*lambda → scale ×1.1)
    const float mag = norm2(s.x1, s.x2);
    if (mag < lambda * 0.5f) {
        s.x1 *= 1.1f;
        s.x2 *= 1.1f;
    }

    s.i_alpha_last = i_alpha;
    s.i_beta_last  = i_beta;

    // For MXLEMMING: L_ia = L_ib = 0, so phase = atan2(x2, x1)
    return fast_atan2(s.x2, s.x1);
}

// ── PLL state and update ──────────────────────────────────────────────────────
// Matches VESC foc_pll_run() — used to estimate electrical speed from angle.
struct PllState {
    float phase{0.0f};   // estimated electrical angle [rad]
    float speed{0.0f};   // estimated electrical speed  [rad/s]
};

inline void pll_run(float phase_input, float dt,
                    float kp, float ki,
                    PllState &s)
{
    nan_zero(s.phase);
    nan_zero(s.speed);
    float delta = phase_input - s.phase;
    norm_angle_rad(delta);
    s.phase += (s.speed + kp * delta) * dt;
    norm_angle_rad(s.phase);
    s.speed += ki * delta * dt;
}

// ── Field-weakening helper ────────────────────────────────────────────────────
// Returns the Id FW current command (negative) based on current duty ratio.
// Matches VESC foc_run_fw() — duty_abs is |mod_q| ≈ |Vq|/(Vbus/sqrt(3)).
// fw_current_max, fw_duty_start, l_max_duty from XML.
inline float fw_compute(
        float duty_abs,
        float fw_duty_start, float l_max_duty,
        float fw_current_max,
        float fw_i_set_prev, float fw_ramp_time, float dt,
        float cc_min_current)
{
    if (fw_current_max < std::max(cc_min_current, 0.001f)) {
        return 0.0f;
    }

    float fw_target = 0.0f;
    if (fw_duty_start < 0.99f &&
        duty_abs > fw_duty_start * l_max_duty)
    {
        // Linear map: duty in [fw_duty_start*l_max_duty, l_max_duty] → [0, fw_current_max]
        float t = (duty_abs - fw_duty_start * l_max_duty) /
                  ((1.0f - fw_duty_start) * l_max_duty);
        truncate(t, 0.0f, 1.0f);
        fw_target = t * fw_current_max;
    }

    // Rate-limit ramp
    float fw_new = fw_i_set_prev;
    if (fw_ramp_time < dt) {
        fw_new = fw_target;
    } else {
        float step = (dt / fw_ramp_time) * fw_current_max;
        float diff = fw_target - fw_new;
        if      (diff >  step) fw_new += step;
        else if (diff < -step) fw_new -= step;
        else                   fw_new  = fw_target;
    }
    return fw_new;
}

} // namespace foc_math
