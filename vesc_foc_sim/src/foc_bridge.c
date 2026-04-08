/*
 * foc_bridge.c — C bridge between vesc_foc_sim and VESC firmware C code.
 *
 * This file is compiled as pure C so datatypes.h (which is C-only compatible)
 * is never seen by the C++ compiler. It constructs the minimal motor_all_state_t
 * and mc_configuration structs needed to call the real firmware functions.
 *
 * Only add bridge functions here for firmware calls the sim actually needs.
 * Everything else (Park, Clarke, utils) stays as inline C++ in adapter.hpp.
 */

#include "foc_math.h"   /* motor/foc_math.h  — foc_observer_update, foc_pll_run */
#include "utils_math.h" /* util/utils_math.h — static inline helpers used above  */
#include <string.h>     /* memset */

/* ── Observer bridge ─────────────────────────────────────────────────────────
 * Wraps foc_observer_update() with a plain scalar interface so C++ code never
 * needs to see observer_state or motor_all_state_t.
 *
 * Observer is fixed to MXLEMMING (foc_observer_type = 1), no saturation
 * compensation, no temperature compensation, no saliency (ld_lq_diff = 0).
 * These are the settings from the FS XML config the sim targets.
 */
float foc_bridge_observer_update(
        float v_alpha, float v_beta,
        float i_alpha, float i_beta,
        float dt,
        float R, float L, float lambda,
        float *x1, float *x2, float *lambda_est,
        float *i_alpha_last, float *i_beta_last)
{
    mc_configuration conf;
    memset(&conf, 0, sizeof(conf));
    conf.foc_motor_r            = R;
    conf.foc_motor_l            = L;
    conf.foc_motor_flux_linkage = lambda;
    conf.foc_observer_type      = FOC_OBSERVER_MXLEMMING;
    conf.foc_sat_comp_mode      = SAT_COMP_DISABLED;
    /* foc_temp_comp = false, foc_motor_ld_lq_diff = 0.0 via memset */

    motor_all_state_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.m_conf = &conf;
    /* m_motor_state.id/iq/i_abs_filter = 0, m_gamma_now = 0 via memset */
    /* (m_gamma_now unused for MXLEMMING; id/iq only matter for saliency) */

    observer_state state;
    state.x1           = *x1;
    state.x2           = *x2;
    state.lambda_est   = *lambda_est;
    state.i_alpha_last = *i_alpha_last;
    state.i_beta_last  = *i_beta_last;

    float phase = 0.0f;
    foc_observer_update(v_alpha, v_beta, i_alpha, i_beta, dt,
                        &state, &phase, &motor);

    *x1           = state.x1;
    *x2           = state.x2;
    *lambda_est   = state.lambda_est;
    *i_alpha_last = state.i_alpha_last;
    *i_beta_last  = state.i_beta_last;

    return phase;
}

/* ── PLL bridge ──────────────────────────────────────────────────────────────
 * Wraps foc_pll_run() — passes kp/ki directly so C++ doesn't need
 * mc_configuration.
 */
void foc_bridge_pll_run(
        float phase_input, float dt, float kp, float ki,
        float *phase_var, float *speed_var)
{
    mc_configuration conf;
    memset(&conf, 0, sizeof(conf));
    conf.foc_pll_kp = kp;
    conf.foc_pll_ki = ki;

    foc_pll_run(phase_input, dt, phase_var, speed_var, &conf);
}
