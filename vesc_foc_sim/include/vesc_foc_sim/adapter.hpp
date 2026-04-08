#pragma once
// adapter.hpp — C++ adapter for VESC firmware C math functions.
//
// Provides the foc_math:: namespace used by the sim nodes, backed by the real
// firmware sources (motor/foc_math.c, util/utils_math.c) via foc_bridge.c.
//
// The C++ nodes never include firmware headers directly — all ChibiOS/RTOS
// types are confined to the C compilation units (foc_bridge.c, foc_math.c).

#include <cmath>
#include <cstdint>
#include <algorithm>

// ── Bridge declarations (implemented in src/foc_bridge.c, compiled as C) ─────
extern "C" {
    // Wraps foc_observer_update() — MXLEMMING mode, SAT_COMP_NONE.
    // State is passed as individual scalars to avoid exposing observer_state
    // (from foc_math.h) to the C++ compiler.
    float foc_bridge_observer_update(
        float v_alpha, float v_beta,
        float i_alpha, float i_beta,
        float dt, float R, float L, float lambda,
        float *x1, float *x2, float *lambda_est,
        float *i_alpha_last, float *i_beta_last);

    // Wraps foc_pll_run() — passes kp/ki as scalars instead of mc_configuration*.
    void foc_bridge_pll_run(
        float phase_input, float dt, float kp, float ki,
        float *phase_var, float *speed_var);
}

// ─────────────────────────────────────────────────────────────────────────────
namespace foc_math {

// ── Constants ────────────────────────────────────────────────────────────────
static constexpr float ONE_BY_SQRT3 = 0.57735026919f;
static constexpr float TWO_BY_SQRT3 = 1.15470053838f;
static constexpr float SQRT3_BY_2   = 0.86602540378f;

// ── Scalar utilities (trivial, kept inline — not worth a bridge call) ────────
inline float sq(float x) { return x * x; }

inline void truncate_abs(float &x, float max_val) {
    if      (x >  max_val) x =  max_val;
    else if (x < -max_val) x = -max_val;
}

inline void truncate(float &x, float lo, float hi) {
    if      (x > hi) x = hi;
    else if (x < lo) x = lo;
}

// Matches UTILS_LP_FAST macro from util/utils_math.h
inline void lp_fast(float &val, float sample, float k) {
    val -= k * (val - sample);
}

// Matches utils_norm_angle_rad (static inline in util/utils_math.h)
inline void norm_angle_rad(float &a) {
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    while (a >=  static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
}

// ── Clarke / Park / inverse Park (inlined in mcpwm_foc.c, trivial here) ─────
inline void clarke(float ia, float ib, float &i_alpha, float &i_beta) {
    i_alpha = ia;
    i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
}

inline void park(float i_alpha, float i_beta,
                 float cos_theta, float sin_theta,
                 float &id, float &iq) {
    id =  cos_theta * i_alpha + sin_theta * i_beta;
    iq =  cos_theta * i_beta  - sin_theta * i_alpha;
}

inline void park_inv(float vd, float vq,
                     float cos_theta, float sin_theta,
                     float &v_alpha, float &v_beta) {
    v_alpha = cos_theta * vd - sin_theta * vq;
    v_beta  = sin_theta * vd + cos_theta * vq;
}

// ── State structs ─────────────────────────────────────────────────────────────
// ObserverState mirrors observer_state from motor/foc_math.h field-for-field.
// lambda_est is used by MXLEMMING_LAMBDA_COMP and Ortega variants; kept here
// so the struct is ready if the observer type is changed in foc_bridge.c.
struct ObserverState {
    float x1{0.0f};
    float x2{0.0f};
    float lambda_est{0.0f};
    float i_alpha_last{0.0f};
    float i_beta_last{0.0f};
};

struct PllState {
    float phase{0.0f};
    float speed{0.0f};
};

// ── Firmware-backed functions (via foc_bridge.c → motor/foc_math.c) ──────────

// Calls foc_observer_update() with MXLEMMING observer, no sat/temp comp.
// Returns estimated electrical angle [rad].
inline float observer_update(
        float v_alpha, float v_beta,
        float i_alpha, float i_beta,
        float dt, float R, float L, float lambda,
        ObserverState &s)
{
    return foc_bridge_observer_update(
        v_alpha, v_beta, i_alpha, i_beta, dt, R, L, lambda,
        &s.x1, &s.x2, &s.lambda_est, &s.i_alpha_last, &s.i_beta_last);
}

// Calls foc_pll_run() for electrical speed estimation from the observer angle.
inline void pll_run(float phase_input, float dt,
                    float kp, float ki,
                    PllState &s)
{
    foc_bridge_pll_run(phase_input, dt, kp, ki, &s.phase, &s.speed);
}

// ── Field weakening ───────────────────────────────────────────────────────────
// foc_run_fw() is deeply tied to the firmware FSM (mc_state, control_mode,
// duty filtering). The logic below is a direct extraction of the math,
// matching foc_run_fw() for CONTROL_MODE_CURRENT with duty-based ramp.
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
        float t = (duty_abs - fw_duty_start * l_max_duty) /
                  ((1.0f - fw_duty_start) * l_max_duty);
        truncate(t, 0.0f, 1.0f);
        fw_target = t * fw_current_max;
    }

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
