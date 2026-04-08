// vesc_controller_node.cpp
// VESC FOC current controller — ported from motor/mcpwm_foc.c (control_current)
// and motor/foc_math.c (observer, PLL, FW).
//
// Phase 1: uses /foc/theta_real (perfect encoder). Set use_observer=false (default).
// Phase 2: set use_observer=true to use MXLEMMING observer + PLL instead.
//
// Subscribed topics:
//   /iq_ref          [Float64]  current setpoint  [A]  (also resets watchdog)
//   /foc/i_alpha     [Float64]  α-frame current   [A]
//   /foc/i_beta      [Float64]  β-frame current   [A]
//   /foc/theta_real  [Float64]  real elec. angle  [rad]  (Phase 1)
//   /foc/omega       [Float64]  real elec. speed  [rad/s]
//   /foc/vbus        [Float64]  bus voltage        [V]
//
// Published topics:
//   /foc/vd          [Float64]  d-axis voltage     [V]
//   /foc/vq          [Float64]  q-axis voltage     [V]
//   /foc/theta_est   [Float64]  observer angle estimate [rad]
//   /foc/pi_d_err    [Float64]  PI d-axis error    [A]  (debug)
//   /foc/pi_q_err    [Float64]  PI q-axis error    [A]  (debug)
//   /foc/id_ref      [Float64]  effective Id reference (0 or FW)
//   /foc/iq_ref_clamped [Float64] Iq ref after limits

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm>
#include "vesc_foc_sim/adapter.hpp"

using F64 = std_msgs::msg::Float64;

// ─────────────────────────────────────────────────────────────────────────────
class VescControllerNode : public rclcpp::Node
{
public:
    VescControllerNode() : Node("vesc_controller")
    {
        // ── Declare parameters ─────────────────────────────────────────────────
        declare_parameter("R",               0.03);
        declare_parameter("Ld",              7.0e-5);
        declare_parameter("Lq",              1.1e-4);
        declare_parameter("lambda",          0.0115);
        declare_parameter("p",               4);
        declare_parameter("current_kp",      0.4);
        declare_parameter("current_ki",      400.0);
        declare_parameter("current_filter",  0.15);
        declare_parameter("observer_gain",   8140000.0);
        declare_parameter("pll_kp",          2000.0);
        declare_parameter("pll_ki",          30000.0);
        declare_parameter("fw_current_max",  48.0);
        declare_parameter("fw_duty_start",   0.9);
        declare_parameter("fw_ramp_time",    0.2);
        declare_parameter("cc_min_current",  0.0);
        declare_parameter("l_current_max",   110.0);
        declare_parameter("l_current_min",  -110.0);
        declare_parameter("l_in_current_max", 80.0);
        declare_parameter("l_in_current_min",-80.0);
        declare_parameter("l_abs_current_max",150.0);
        declare_parameter("l_max_erpm",      70000.0);
        declare_parameter("l_min_vin",       35.0);
        declare_parameter("l_max_vin",       70.0);
        declare_parameter("l_max_duty",      0.95);
        declare_parameter("sim_dt_s",        1.0e-4);
        declare_parameter("vbus",            51.8);
        declare_parameter("timeout_s",       0.1);
        declare_parameter("use_observer",    false);

        // ── mc_interface override limits ────────────────────────────────────────
        declare_parameter("t_fet",                    25.0);
        declare_parameter("t_motor",                  25.0);
        declare_parameter("l_temp_fet_start",         85.0);
        declare_parameter("l_temp_fet_end",          100.0);
        declare_parameter("l_temp_motor_start",       85.0);
        declare_parameter("l_temp_motor_end",        100.0);
        declare_parameter("l_temp_accel_dec",          0.15);
        declare_parameter("l_battery_cut_start",      47.0);
        declare_parameter("l_battery_cut_end",        42.0);
        declare_parameter("l_battery_regen_cut_start", 1000.0);
        declare_parameter("l_battery_regen_cut_end",   1100.0);
        declare_parameter("l_watt_max",             1500000.0);
        declare_parameter("l_watt_min",            -1500000.0);
        declare_parameter("l_duty_start",              1.0);
        declare_parameter("l_erpm_start",              1.0);
        declare_parameter("l_min_erpm",           -70000.0);
        declare_parameter("l_in_current_map_start",    1.0);
        declare_parameter("l_in_current_map_filter",   0.01);
        declare_parameter("foc_start_curr_dec",        1.0);
        declare_parameter("foc_start_curr_dec_rpm",  2500.0);

        // ── Read parameters ────────────────────────────────────────────────────
        lambda_   = static_cast<float>(get_parameter("lambda").as_double());
        Ld_       = static_cast<float>(get_parameter("Ld").as_double());
        Lq_       = static_cast<float>(get_parameter("Lq").as_double());
        R_        = static_cast<float>(get_parameter("R").as_double());
        p_        = get_parameter("p").as_int();
        // L is the mean inductance used by the observer; derive from Ld/Lq
        L_ = (Ld_ + Lq_) * 0.5f;
        kp_       = static_cast<float>(get_parameter("current_kp").as_double());
        ki_       = static_cast<float>(get_parameter("current_ki").as_double());
        kf_       = static_cast<float>(get_parameter("current_filter").as_double());
        obs_gain_ = static_cast<float>(get_parameter("observer_gain").as_double());
        pll_kp_   = static_cast<float>(get_parameter("pll_kp").as_double());
        pll_ki_   = static_cast<float>(get_parameter("pll_ki").as_double());
        fw_i_max_ = static_cast<float>(get_parameter("fw_current_max").as_double());
        fw_duty_start_ = static_cast<float>(get_parameter("fw_duty_start").as_double());
        fw_ramp_  = static_cast<float>(get_parameter("fw_ramp_time").as_double());
        cc_min_i_ = static_cast<float>(get_parameter("cc_min_current").as_double());
        i_max_    = static_cast<float>(get_parameter("l_current_max").as_double());
        i_min_    = static_cast<float>(get_parameter("l_current_min").as_double());
        i_in_max_ = static_cast<float>(get_parameter("l_in_current_max").as_double());
        i_in_min_ = static_cast<float>(get_parameter("l_in_current_min").as_double());
        i_abs_max_= static_cast<float>(get_parameter("l_abs_current_max").as_double());
        erpm_max_ = static_cast<float>(get_parameter("l_max_erpm").as_double());
        v_min_    = static_cast<float>(get_parameter("l_min_vin").as_double());
        v_max_    = static_cast<float>(get_parameter("l_max_vin").as_double());
        max_duty_ = static_cast<float>(get_parameter("l_max_duty").as_double());
        dt_       = static_cast<float>(get_parameter("sim_dt_s").as_double());
        vbus_     = static_cast<float>(get_parameter("vbus").as_double());
        timeout_s_= static_cast<float>(get_parameter("timeout_s").as_double());
        use_obs_  = get_parameter("use_observer").as_bool();

        t_fet_                   = static_cast<float>(get_parameter("t_fet").as_double());
        t_motor_                 = static_cast<float>(get_parameter("t_motor").as_double());
        l_temp_fet_start_        = static_cast<float>(get_parameter("l_temp_fet_start").as_double());
        l_temp_fet_end_          = static_cast<float>(get_parameter("l_temp_fet_end").as_double());
        l_temp_motor_start_      = static_cast<float>(get_parameter("l_temp_motor_start").as_double());
        l_temp_motor_end_        = static_cast<float>(get_parameter("l_temp_motor_end").as_double());
        l_temp_accel_dec_        = static_cast<float>(get_parameter("l_temp_accel_dec").as_double());
        l_battery_cut_start_     = static_cast<float>(get_parameter("l_battery_cut_start").as_double());
        l_battery_cut_end_       = static_cast<float>(get_parameter("l_battery_cut_end").as_double());
        l_battery_regen_cut_start_ = static_cast<float>(get_parameter("l_battery_regen_cut_start").as_double());
        l_battery_regen_cut_end_ = static_cast<float>(get_parameter("l_battery_regen_cut_end").as_double());
        l_watt_max_              = static_cast<float>(get_parameter("l_watt_max").as_double());
        l_watt_min_              = static_cast<float>(get_parameter("l_watt_min").as_double());
        l_duty_start_            = static_cast<float>(get_parameter("l_duty_start").as_double());
        l_erpm_start_            = static_cast<float>(get_parameter("l_erpm_start").as_double());
        l_min_erpm_              = static_cast<float>(get_parameter("l_min_erpm").as_double());
        l_in_current_map_start_  = static_cast<float>(get_parameter("l_in_current_map_start").as_double());
        l_in_current_map_filter_ = static_cast<float>(get_parameter("l_in_current_map_filter").as_double());
        foc_start_curr_dec_      = static_cast<float>(get_parameter("foc_start_curr_dec").as_double());
        foc_start_curr_dec_rpm_  = static_cast<float>(get_parameter("foc_start_curr_dec_rpm").as_double());

        RCLCPP_INFO(get_logger(),
            "Controller — Kp=%.3f Ki=%.1f dt=%.1fµs use_observer=%s",
            kp_, ki_, dt_ * 1e6f, use_obs_ ? "true" : "false");

        // ── Subscribers ────────────────────────────────────────────────────────
        auto qos = rclcpp::SystemDefaultsQoS();
        sub_iq_ref_  = create_subscription<F64>("/iq_ref",         qos,
            std::bind(&VescControllerNode::on_iq_ref,    this, std::placeholders::_1));
        sub_ialpha_  = create_subscription<F64>("/foc/i_alpha",    qos,
            [this](const F64::SharedPtr m){ i_alpha_ = static_cast<float>(m->data); });
        sub_ibeta_   = create_subscription<F64>("/foc/i_beta",     qos,
            [this](const F64::SharedPtr m){ i_beta_  = static_cast<float>(m->data); });
        sub_theta_   = create_subscription<F64>("/foc/theta_real", qos,
            [this](const F64::SharedPtr m){ theta_real_ = static_cast<float>(m->data); });
        sub_omega_   = create_subscription<F64>("/foc/omega",      qos,
            [this](const F64::SharedPtr m){ omega_e_ = static_cast<float>(m->data); });
        sub_vbus_    = create_subscription<F64>("/foc/vbus",       qos,
            [this](const F64::SharedPtr m){ vbus_    = static_cast<float>(m->data); });

        // ── Publishers ─────────────────────────────────────────────────────────
        auto lat = rclcpp::SensorDataQoS();
        pub_vd_        = create_publisher<F64>("/foc/vd",           lat);
        pub_vq_        = create_publisher<F64>("/foc/vq",           lat);
        pub_theta_est_ = create_publisher<F64>("/foc/theta_est",    lat);
        pub_pi_d_err_  = create_publisher<F64>("/foc/pi_d_err",     lat);
        pub_pi_q_err_  = create_publisher<F64>("/foc/pi_q_err",     lat);
        pub_id_ref_    = create_publisher<F64>("/foc/id_ref",       lat);
        pub_iq_clamped_= create_publisher<F64>("/foc/iq_ref_clamped", lat);

        pub_lo_i_max_  = create_publisher<F64>("/foc/lo_current_max", lat);
        pub_fault_     = create_publisher<F64>("/foc/fault",          lat);

        // ── Timer ──────────────────────────────────────────────────────────────
        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(dt_ * 1e9));
        timer_ = create_wall_timer(period_ns,
            std::bind(&VescControllerNode::control_step, this));
    }

private:
    // ── iq_ref subscriber ─────────────────────────────────────────────────────
    void on_iq_ref(const F64::SharedPtr msg) {
        iq_ref_raw_    = static_cast<float>(msg->data);
        watchdog_timer_ = 0.0f;   // reset watchdog
    }

    // ── Main control step — runs at sim_dt_s ──────────────────────────────────
    void control_step()
    {
        // ── Watchdog ───────────────────────────────────────────────────────────
        watchdog_timer_ += dt_;
        if (watchdog_timer_ > timeout_s_) {
            iq_ref_raw_ = 0.0f;   // coast
        }

        // ── Bus voltage protection ─────────────────────────────────────────────
        if (vbus_ < v_min_ || vbus_ > v_max_) {
            // Inhibit: publish zero voltages and return
            publish(pub_vd_, 0.0f);
            publish(pub_vq_, 0.0f);
            return;
        }

        // ── Speed limit ────────────────────────────────────────────────────────
        // omega_e_ is electrical [rad/s]; convert to ERPM = omega_e * 60 / (2π)
        const float erpm = omega_e_ * 60.0f / (2.0f * static_cast<float>(M_PI));
        if (std::fabs(erpm) > erpm_max_) {
            iq_ref_raw_ = 0.0f;
        }

        // ── mc_interface override limits ────────────────────────────────────────
        foc_math::lp_fast(i_in_filter_,
            (vbus_ > 0.0f) ? (vd_out_ * id_filt_ + vq_out_ * iq_filt_) / vbus_ : 0.0f,
            l_in_current_map_filter_);

        const float erpm_now = omega_e_ * 60.0f / (2.0f * static_cast<float>(M_PI));
        const float v_norm_duty = vbus_ * foc_math::ONE_BY_SQRT3;
        const float duty_abs_now = (v_norm_duty > 0.0f)
            ? std::fabs(vq_out_) / v_norm_duty : 0.0f;

        const foc_math::OverrideLimitResult lim = foc_math::compute_override_limits({
            i_max_, i_min_,
            i_in_max_, i_in_min_,
            t_fet_, t_motor_,
            l_temp_fet_start_, l_temp_fet_end_,
            l_temp_motor_start_, l_temp_motor_end_,
            l_temp_accel_dec_,
            vbus_,
            l_battery_cut_start_, l_battery_cut_end_,
            l_battery_regen_cut_start_, l_battery_regen_cut_end_,
            l_watt_max_, l_watt_min_,
            erpm_now,
            erpm_max_, l_min_erpm_, l_erpm_start_,
            foc_start_curr_dec_, foc_start_curr_dec_rpm_,
            duty_abs_now,
            max_duty_, l_duty_start_,
            i_in_filter_,
            l_in_current_map_start_,
            cc_min_i_
        });

        float iq_ref = iq_ref_raw_;
        foc_math::truncate(iq_ref, lim.lo_current_min, lim.lo_current_max);

        // ── Angle selection ────────────────────────────────────────────────────
        float theta_use;
        if (use_obs_) {
            // Phase 2: update observer and PLL from αβ measurements
            const float theta_obs = foc_math::observer_update(
                vd_alpha_last_, vq_beta_last_,   // last applied v_alpha/v_beta
                i_alpha_, i_beta_,
                dt_, R_, L_, lambda_, obs_state_);
            foc_math::pll_run(theta_obs, dt_, pll_kp_, pll_ki_, pll_);
            theta_use = pll_.phase;
        } else {
            // Phase 1: perfect angle from plant
            theta_use = theta_real_;
            pll_.phase = theta_real_;       // keep in sync for future Phase 2 switch
            pll_.speed = omega_e_;
        }

        const float cos_t = std::cos(theta_use);
        const float sin_t = std::sin(theta_use);

        // ── Park transform: αβ → dq ───────────────────────────────────────────
        float id_meas, iq_meas;
        foc_math::park(i_alpha_, i_beta_, cos_t, sin_t, id_meas, iq_meas);

        // ── Low-pass filters (for decoupling feedforward only) ─────────────────
        foc_math::lp_fast(id_filt_, id_meas, kf_);
        foc_math::lp_fast(iq_filt_, iq_meas, kf_);

        // ── Field weakening: compute Id reference ─────────────────────────────
        // Use |mod_q| as duty proxy: mod_q = vq / (vbus / sqrt(3))
        // We approximate mod_q with the last output.
        const float v_norm = vbus_ * foc_math::ONE_BY_SQRT3;   // V_max at unity mod
        const float duty_abs = (v_norm > 0.0f)
            ? std::fabs(vq_out_) / v_norm : 0.0f;

        fw_i_set_ = foc_math::fw_compute(
            duty_abs,
            fw_duty_start_, max_duty_,
            fw_i_max_, fw_i_set_, fw_ramp_, dt_,
            cc_min_i_);

        // id_ref: 0 normally, or negative for FW
        float id_ref = -fw_i_set_;

        // ── PI controllers — matches VESC control_current() lines 4607-4620 ───
        const float Ierr_d = id_ref  - id_meas;
        const float Ierr_q = iq_ref  - iq_meas;

        vd_int_ += Ierr_d * ki_ * dt_;
        vq_int_ += Ierr_q * ki_ * dt_;

        float vd = vd_int_ + Ierr_d * kp_;
        float vq = vq_int_ + Ierr_q * kp_;

        // ── Cross-coupling + back-EMF decoupling (CROSS_BEMF mode) ─────────────
        // From XML: foc_cc_decoupling = 3 = FOC_CC_DECOUPLING_CROSS_BEMF
        // dec_vd   = iq_filt · ωe · Lq   (subtract from vd)
        // dec_vq   = id_filt · ωe · Ld   (add to vq)
        // dec_bemf = ωe · λ              (add to vq)
        const float dec_vd   = iq_filt_ * omega_e_ * Lq_;
        const float dec_vq   = id_filt_ * omega_e_ * Ld_;
        const float dec_bemf = omega_e_ * lambda_;

        vd -= dec_vd;
        vq += dec_vq + dec_bemf;

        // ── Voltage saturation + anti-windup ──────────────────────────────────
        // max_v_mag = (1/√3) · max_duty · V_bus   (without overmod, factor=1 from XML)
        const float max_v_mag = foc_math::ONE_BY_SQRT3 * max_duty_ * vbus_;

        foc_math::truncate_abs(vd,     max_v_mag);
        foc_math::truncate_abs(vd_int_, max_v_mag);

        const float max_vq = std::sqrt(std::max(0.0f,
            foc_math::sq(max_v_mag) - foc_math::sq(vd)));

        foc_math::truncate_abs(vq,     max_vq);
        foc_math::truncate_abs(vq_int_, max_vq);

        vd_out_ = vd;
        vq_out_ = vq;

        // ── Store v_alpha/v_beta for observer on next step ─────────────────────
        if (use_obs_) {
            foc_math::park_inv(vd, vq, cos_t, sin_t,
                               vd_alpha_last_, vq_beta_last_);
        }

        // ── Publish outputs ────────────────────────────────────────────────────
        publish(pub_vd_,         vd);
        publish(pub_vq_,         vq);
        publish(pub_theta_est_,  theta_use);
        publish(pub_pi_d_err_,   Ierr_d);
        publish(pub_pi_q_err_,   Ierr_q);
        publish(pub_id_ref_,     id_ref);
        publish(pub_iq_clamped_, iq_ref);
        publish(pub_lo_i_max_, lim.lo_current_max);
        publish(pub_fault_,    (lim.fault_over_temp_fet || lim.fault_over_temp_motor) ? 1.0f : 0.0f);
    }

    static void publish(rclcpp::Publisher<F64>::SharedPtr &pub, float val) {
        F64 msg;
        msg.data = static_cast<double>(val);
        pub->publish(msg);
    }

    // ── Motor params ──────────────────────────────────────────────────────────
    float R_{}, L_{}, Ld_{}, Lq_{}, lambda_{};
    int   p_{4};

    // ── Controller tuning ─────────────────────────────────────────────────────
    float kp_{}, ki_{}, kf_{};
    float obs_gain_{}, pll_kp_{}, pll_ki_{};
    float fw_i_max_{}, fw_duty_start_{}, fw_ramp_{}, cc_min_i_{};

    // ── Limits ────────────────────────────────────────────────────────────────
    float i_max_{}, i_min_{}, i_in_max_{}, i_in_min_{}, i_abs_max_{};
    float erpm_max_{}, v_min_{}, v_max_{}, max_duty_{};

    // ── Sim config ────────────────────────────────────────────────────────────
    float dt_{1e-4f}, vbus_{51.8f}, timeout_s_{0.1f};
    bool  use_obs_{false};

    // ── Sensor inputs (updated by subscribers) ────────────────────────────────
    float i_alpha_{0.0f}, i_beta_{0.0f};
    float theta_real_{0.0f}, omega_e_{0.0f};

    // ── Setpoint ──────────────────────────────────────────────────────────────
    float iq_ref_raw_{0.0f};

    // ── Controller state ──────────────────────────────────────────────────────
    float vd_int_{0.0f}, vq_int_{0.0f};
    float id_filt_{0.0f}, iq_filt_{0.0f};
    float fw_i_set_{0.0f};
    float vd_out_{0.0f}, vq_out_{0.0f};

    // Last applied voltage in αβ (needed by observer)
    float vd_alpha_last_{0.0f}, vq_beta_last_{0.0f};

    // ── Observer / PLL state ──────────────────────────────────────────────────
    foc_math::ObserverState obs_state_{};
    foc_math::PllState      pll_{};

    // ── Watchdog ──────────────────────────────────────────────────────────────
    float watchdog_timer_{0.0f};

    // ── mc_interface limit params ─────────────────────────────────────────────
    float t_fet_{25.0f},   t_motor_{25.0f};
    float l_temp_fet_start_{85.0f},   l_temp_fet_end_{100.0f};
    float l_temp_motor_start_{85.0f}, l_temp_motor_end_{100.0f};
    float l_temp_accel_dec_{0.15f};
    float l_battery_cut_start_{47.0f}, l_battery_cut_end_{42.0f};
    float l_battery_regen_cut_start_{1000.0f}, l_battery_regen_cut_end_{1100.0f};
    float l_watt_max_{1.5e6f}, l_watt_min_{-1.5e6f};
    float l_duty_start_{1.0f}, l_erpm_start_{1.0f}, l_min_erpm_{-70000.0f};
    float l_in_current_map_start_{1.0f}, l_in_current_map_filter_{0.01f};
    float foc_start_curr_dec_{1.0f}, foc_start_curr_dec_rpm_{2500.0f};
    float i_in_filter_{0.0f};

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Subscription<F64>::SharedPtr sub_iq_ref_, sub_ialpha_, sub_ibeta_,
                                          sub_theta_, sub_omega_, sub_vbus_;
    rclcpp::Publisher<F64>::SharedPtr    pub_vd_, pub_vq_, pub_theta_est_,
                                          pub_pi_d_err_, pub_pi_q_err_,
                                          pub_id_ref_, pub_iq_clamped_;
    rclcpp::Publisher<F64>::SharedPtr    pub_lo_i_max_, pub_fault_;
    rclcpp::TimerBase::SharedPtr         timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VescControllerNode>());
    rclcpp::shutdown();
    return 0;
}
