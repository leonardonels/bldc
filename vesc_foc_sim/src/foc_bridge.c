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
#include "utils_math.h" /* util/utils_math.h — utils_map, utils_min_abs, utils_fast_sincos_better */
#include <math.h>       /* sinf, cosf, fabsf, sqrtf */
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

/* ── Virtual motor step bridge ───────────────────────────────────────────────
 * Ported from motor/virtual_motor.c: run_virtual_motor_electrical(),
 * run_virtual_motor_mechanics(), and the inverse Park from
 * run_virtual_motor_park_clark_inverse().
 *
 * Differences from the original virtual_motor.c:
 *   - Input is vd/vq (dq frame) — no input Park transform needed since the
 *     controller already publishes dq voltages.
 *   - theta_e is a true electrical angle (theta_e += p*omega_m*dt), correcting
 *     a latent multi-pole bug in virtual_motor.c where phi accumulated as
 *     mechanical angle but was used in Park transforms.
 *   - Viscous damping is omitted (matches virtual_motor.c).
 *   - Reluctance torque included: τe = (3/2)·p·(λ + (Ld−Lq)·id)·iq
 *
 * State fields (caller owns, persists across calls):
 *   id_int   — d-axis integral state [A]
 *   id, iq   — dq currents [A]
 *   omega_m  — mechanical speed [rad/s]
 *   theta_e  — electrical angle [rad]
 */
void vm_step_bridge(
        float *id_int, float *id, float *iq,
        float *omega_m, float *theta_e,
        float vd, float vq, float dt,
        float R, float Ld, float Lq, float lambda,
        float J, float tau_load, int pole_pairs,
        float *i_alpha_out, float *i_beta_out, float *torque_out)
{
    const float omega_e = (float)pole_pairs * (*omega_m);

    /* Electrical ODE — Euler, from run_virtual_motor_electrical() */
    *id_int += (vd + omega_e * Lq * (*iq) - R * (*id)) * dt / Ld;
    *id      = *id_int - lambda / Ld;
    *iq     += (vq - omega_e * (Ld * (*id) + lambda) - R * (*iq)) * dt / Lq;

    /* Mechanics — from run_virtual_motor_mechanics() */
    const float km  = 1.5f * (float)pole_pairs;
    const float tau = km * (lambda + (Ld - Lq) * (*id)) * (*iq);
    *omega_m += (dt / J) * (tau - tau_load);

    /* Electrical angle */
    *theta_e += omega_e * dt;
    utils_norm_angle_rad(theta_e);

    /* Inverse Park → αβ currents — from run_virtual_motor_park_clark_inverse() */
    float s, c;
    utils_fast_sincos_better(*theta_e, &s, &c);
    *i_alpha_out = c * (*id) - s * (*iq);
    *i_beta_out  = s * (*id) + c * (*iq);
    *torque_out  = tau;
}

/* ── mc_interface override limits bridge ─────────────────────────────────────
 * Ported from motor/mc_interface.c :: update_override_limits() (lines 2314–2511).
 *
 * Temperature is a caller-supplied fixed value (no ADC reads).
 * BMS limits are omitted (no-op, matching the sim's standalone use case).
 * l_current_max_scale / l_current_min_scale assumed 1.0 (matches XML).
 *
 * All struct fields are plain C floats/ints — layout is identical to the
 * mirrored C++ structs in adapter.hpp.
 */

typedef struct {
    float i_max, i_min;
    float l_in_current_max, l_in_current_min;
    float t_fet, t_motor;
    float l_temp_fet_start, l_temp_fet_end;
    float l_temp_motor_start, l_temp_motor_end;
    float l_temp_accel_dec;
    float v_in;
    float l_battery_cut_start, l_battery_cut_end;
    float l_battery_regen_cut_start, l_battery_regen_cut_end;
    float l_watt_max, l_watt_min;
    float rpm_now;
    float l_max_erpm, l_min_erpm, l_erpm_start;
    float foc_start_curr_dec, foc_start_curr_dec_rpm;
    float duty_now_abs;
    float l_max_duty, l_duty_start;
    float i_in_filter;
    float l_in_current_map_start;
    float cc_min_current;
} limits_params_t;

typedef struct {
    float lo_current_max;
    float lo_current_min;
    float lo_in_current_max;
    float lo_in_current_min;
    int   fault_over_temp_fet;
    int   fault_over_temp_motor;
} limits_result_t;

void compute_limits_bridge(const limits_params_t *p, limits_result_t *r)
{
    const float rpm_abs = fabsf(p->rpm_now);

    r->fault_over_temp_fet   = 0;
    r->fault_over_temp_motor = 0;

    /* ── Temperature MOSFET ───────────────────────────────────────────────── */
    float lo_min_mos = p->i_min;
    float lo_max_mos = p->i_max;
    if (p->t_fet >= (p->l_temp_fet_end - 0.1f)) {
        lo_min_mos = 0.0f;
        lo_max_mos = 0.0f;
        r->fault_over_temp_fet = 1;
    } else if (p->t_fet > (p->l_temp_fet_start + 0.1f)) {
        float maxc = fabsf(p->i_max);
        if (fabsf(p->i_min) > maxc) maxc = fabsf(p->i_min);
        maxc = utils_map(p->t_fet, p->l_temp_fet_start, p->l_temp_fet_end, maxc, 0.0f);
        if (fabsf(p->i_min) > maxc) lo_min_mos = (p->i_min < 0.0f ? -1.0f : 1.0f) * maxc;
        if (fabsf(p->i_max) > maxc) lo_max_mos = (p->i_max < 0.0f ? -1.0f : 1.0f) * maxc;
    }

    /* ── Temperature MOTOR ────────────────────────────────────────────────── */
    float lo_min_mot = p->i_min;
    float lo_max_mot = p->i_max;
    if (p->t_motor >= (p->l_temp_motor_end - 0.1f)) {
        lo_min_mot = 0.0f;
        lo_max_mot = 0.0f;
        r->fault_over_temp_motor = 1;
    } else if (p->t_motor > (p->l_temp_motor_start + 0.1f)) {
        float maxc = fabsf(p->i_max);
        if (fabsf(p->i_min) > maxc) maxc = fabsf(p->i_min);
        maxc = utils_map(p->t_motor, p->l_temp_motor_start, p->l_temp_motor_end, maxc, 0.0f);
        if (fabsf(p->i_min) > maxc) lo_min_mot = (p->i_min < 0.0f ? -1.0f : 1.0f) * maxc;
        if (fabsf(p->i_max) > maxc) lo_max_mot = (p->i_max < 0.0f ? -1.0f : 1.0f) * maxc;
    }

    /* ── Accel derating ───────────────────────────────────────────────────── */
    const float tf_acc_start = utils_map(p->l_temp_accel_dec, 0.0f, 1.0f, p->l_temp_fet_start,   25.0f);
    const float tf_acc_end   = utils_map(p->l_temp_accel_dec, 0.0f, 1.0f, p->l_temp_fet_end,     25.0f);
    const float tm_acc_start = utils_map(p->l_temp_accel_dec, 0.0f, 1.0f, p->l_temp_motor_start, 25.0f);
    const float tm_acc_end   = utils_map(p->l_temp_accel_dec, 0.0f, 1.0f, p->l_temp_motor_end,   25.0f);

    float lo_fet_temp_accel = p->i_max;
    if      (p->t_fet >= (tf_acc_end   - 0.1f)) lo_fet_temp_accel = 0.0f;
    else if (p->t_fet >  (tf_acc_start + 0.1f))
        lo_fet_temp_accel = utils_map(p->t_fet, tf_acc_start, tf_acc_end, p->i_max, 0.0f);

    float lo_mot_temp_accel = p->i_max;
    if      (p->t_motor >= (tm_acc_end   - 0.1f)) lo_mot_temp_accel = 0.0f;
    else if (p->t_motor >  (tm_acc_start + 0.1f))
        lo_mot_temp_accel = utils_map(p->t_motor, tm_acc_start, tm_acc_end, p->i_max, 0.0f);

    /* ── RPM max ──────────────────────────────────────────────────────────── */
    const float rpm_pos_cut_start = p->l_max_erpm * p->l_erpm_start;
    float lo_max_rpm = p->i_max;
    if      (p->rpm_now >= (p->l_max_erpm - 0.1f))      lo_max_rpm = 0.0f;
    else if (p->rpm_now >  (rpm_pos_cut_start + 0.1f))
        lo_max_rpm = utils_map(p->rpm_now, rpm_pos_cut_start, p->l_max_erpm, p->i_max, 0.0f);

    /* ── RPM min ──────────────────────────────────────────────────────────── */
    const float rpm_neg_cut_start = p->l_min_erpm * p->l_erpm_start;
    float lo_min_rpm = p->i_max;
    if      (p->rpm_now <= (p->l_min_erpm + 0.1f))      lo_min_rpm = 0.0f;
    else if (p->rpm_now <  (rpm_neg_cut_start - 0.1f))
        lo_min_rpm = utils_map(p->rpm_now, rpm_neg_cut_start, p->l_min_erpm, p->i_max, 0.0f);

    /* ── Start current decrease ───────────────────────────────────────────── */
    float lo_max_curr_dec = p->i_max;
    if (rpm_abs < p->foc_start_curr_dec_rpm)
        lo_max_curr_dec = utils_map(rpm_abs, 0.0f, p->foc_start_curr_dec_rpm,
                                    p->foc_start_curr_dec * p->i_max, p->i_max);

    /* ── Duty max (disabled when l_duty_start >= 0.99) ───────────────────── */
    float lo_max_duty = p->i_max;
    if (p->l_duty_start < 0.99f &&
        p->duty_now_abs >= (p->l_duty_start * p->l_max_duty))
        lo_max_duty = utils_map(p->duty_now_abs,
                                p->l_duty_start * p->l_max_duty, p->l_max_duty,
                                p->i_max, p->cc_min_current * 5.0f);

    /* ── Battery cutoff ───────────────────────────────────────────────────── */
    float lo_in_max_batt = p->l_in_current_max;
    if      (p->v_in <= (p->l_battery_cut_end   + 0.1f)) lo_in_max_batt = 0.0f;
    else if (p->v_in <  (p->l_battery_cut_start - 0.1f))
        lo_in_max_batt = utils_map(p->v_in,
                                   p->l_battery_cut_end, p->l_battery_cut_start,
                                   0.0f, p->l_in_current_max);

    /* ── Regen overvoltage cutoff ─────────────────────────────────────────── */
    float lo_in_min_batt = p->l_in_current_min;
    if      (p->v_in >= (p->l_battery_regen_cut_end   - 0.1f)) lo_in_min_batt = 0.0f;
    else if (p->v_in >  (p->l_battery_regen_cut_start + 0.1f))
        lo_in_min_batt = utils_map(p->v_in,
                                   p->l_battery_regen_cut_start, p->l_battery_regen_cut_end,
                                   p->l_in_current_min, 0.0f);

    /* ── Wattage limits ───────────────────────────────────────────────────── */
    const float v_safe        = (p->v_in > 1.0f) ? p->v_in : 1.0f;
    const float lo_in_max_watt = p->l_watt_max / v_safe;
    const float lo_in_min_watt = p->l_watt_min / v_safe;

    const float lo_in_max = utils_min_abs(lo_in_max_watt, lo_in_max_batt);
    const float lo_in_min = utils_min_abs(lo_in_min_watt, lo_in_min_batt);

    /* Finalise input current limits first — read back in lo_max_i_in below */
    r->lo_in_current_max = utils_min_abs(p->l_in_current_max, lo_in_max);
    r->lo_in_current_min = utils_min_abs(p->l_in_current_min, lo_in_min);

    /* ── Input current → iq limit ─────────────────────────────────────────── */
    float lo_max_i_in = p->i_max;
    if (p->i_in_filter > 0.0f &&
        p->l_in_current_map_start < 0.98f &&
        r->lo_in_current_max > 0.0f) {
        const float frac = p->i_in_filter / r->lo_in_current_max;
        if (frac > p->l_in_current_map_start)
            lo_max_i_in = utils_map(frac, p->l_in_current_map_start, 1.0f, p->i_max, 0.0f);
        if (lo_max_i_in < 0.0f) lo_max_i_in = 0.0f;
    }

    /* ── Waterfall: take most restrictive limit ───────────────────────────── */
    float lo_max = utils_min_abs(lo_max_mos, lo_max_mot);
    float lo_min = utils_min_abs(lo_min_mos, lo_min_mot);
    lo_max = utils_min_abs(lo_max, lo_max_rpm);
    lo_max = utils_min_abs(lo_max, lo_min_rpm);
    lo_max = utils_min_abs(lo_max, lo_max_curr_dec);
    lo_max = utils_min_abs(lo_max, lo_fet_temp_accel);
    lo_max = utils_min_abs(lo_max, lo_mot_temp_accel);
    lo_max = utils_min_abs(lo_max, lo_max_duty);
    lo_max = utils_min_abs(lo_max, lo_max_i_in);

    r->lo_current_max = (lo_max > p->cc_min_current) ? lo_max : p->cc_min_current;
    r->lo_current_min = (lo_min < -p->cc_min_current) ? lo_min : -p->cc_min_current;
}
