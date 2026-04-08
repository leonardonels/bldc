// pmsm_plant_node.cpp
// PMSM physical plant: integrates dq ODE with RK4 and publishes electrical state.
//
// ODE (rotor reference frame, SPM assumption λd=λ, λq=0):
//   dId/dt  = (Vd − R·Id + ωe·Lq·Iq) / Ld
//   dIq/dt  = (Vq − R·Iq − ωe·Ld·Id − ωe·λ) / Lq
//   dωm/dt  = (τe − τload − B·ωm) / J
//   dθe/dt  = p · ωm
// where  τe = (3/2)·p·λ·Iq   ωe = p·ωm
//
// Subscribed topics:
//   /foc/vd        [std_msgs/Float64]  — d-axis voltage from controller
//   /foc/vq        [std_msgs/Float64]  — q-axis voltage from controller
//
// Published topics:
//   /foc/id        [Float64]   d-axis current  [A]
//   /foc/iq        [Float64]   q-axis current  [A]
//   /foc/i_alpha   [Float64]   α-frame current [A]  (for controller Park in Phase 2)
//   /foc/i_beta    [Float64]   β-frame current [A]
//   /foc/theta_real[Float64]   electrical angle [rad]
//   /foc/omega     [Float64]   electrical speed [rad/s]
//   /foc/torque    [Float64]   mechanical torque [Nm]

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include "vesc_foc_sim/adapter.hpp"

using std::placeholders::_1;
using F64 = std_msgs::msg::Float64;

// ── Plant state ───────────────────────────────────────────────────────────────
struct State {
    float id{0.0f};       // d-axis current [A]
    float iq{0.0f};       // q-axis current [A]
    float omega_m{0.0f};  // mechanical speed [rad/s]
    float theta_e{0.0f};  // electrical angle [rad]

    State operator+(const State &o) const {
        return {id+o.id, iq+o.iq, omega_m+o.omega_m, theta_e+o.theta_e};
    }
    State operator*(float s) const {
        return {id*s, iq*s, omega_m*s, theta_e*s};
    }
};

// ── Motor parameters (loaded from YAML) ───────────────────────────────────────
struct MotorParams {
    float R, Ld, Lq, lambda;
    int   p;
    float J_tot, B, tau_load;
};

// ── Derivative of the state ────────────────────────────────────────────────────
static State deriv(const State &s, float vd, float vq, const MotorParams &m) {
    const float omega_e = static_cast<float>(m.p) * s.omega_m;
    const float tau_e   = 1.5f * static_cast<float>(m.p) * m.lambda * s.iq;

    return {
        (vd - m.R * s.id  + omega_e * m.Lq * s.iq) / m.Ld,
        (vq - m.R * s.iq  - omega_e * m.Ld * s.id - omega_e * m.lambda) / m.Lq,
        (tau_e - m.tau_load - m.B * s.omega_m) / m.J_tot,
        static_cast<float>(m.p) * s.omega_m
    };
}

// ── RK4 integrator ─────────────────────────────────────────────────────────────
static void rk4_step(State &s, float vd, float vq, const MotorParams &m, float dt) {
    const State k1 = deriv(s,                        vd, vq, m);
    const State k2 = deriv(s + k1 * (dt * 0.5f),    vd, vq, m);
    const State k3 = deriv(s + k2 * (dt * 0.5f),    vd, vq, m);
    const State k4 = deriv(s + k3 * dt,              vd, vq, m);
    s = s + (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (dt / 6.0f);

    // Wrap angle
    foc_math::norm_angle_rad(s.theta_e);
}

// ─────────────────────────────────────────────────────────────────────────────
class PmsmPlantNode : public rclcpp::Node
{
public:
    PmsmPlantNode() : Node("pmsm_plant")
    {
        // ── Declare & read parameters ──────────────────────────────────────────
        declare_parameter("R",        0.03);
        declare_parameter("Ld",       7.0e-5);
        declare_parameter("Lq",       1.1e-4);
        declare_parameter("lambda",   0.0115);
        declare_parameter("p",        4);
        declare_parameter("J_tot",    7.5e-4);
        declare_parameter("B",        0.0);
        declare_parameter("tau_load", 0.0);
        declare_parameter("sim_dt_s", 1.0e-4);

        mp_.R        = static_cast<float>(get_parameter("R").as_double());
        mp_.Ld       = static_cast<float>(get_parameter("Ld").as_double());
        mp_.Lq       = static_cast<float>(get_parameter("Lq").as_double());
        mp_.lambda   = static_cast<float>(get_parameter("lambda").as_double());
        mp_.p        = get_parameter("p").as_int();
        mp_.J_tot    = static_cast<float>(get_parameter("J_tot").as_double());
        mp_.B        = static_cast<float>(get_parameter("B").as_double());
        mp_.tau_load = static_cast<float>(get_parameter("tau_load").as_double());
        dt_          = static_cast<float>(get_parameter("sim_dt_s").as_double());

        RCLCPP_INFO(get_logger(),
            "Plant — R=%.4f Ld=%.2e Lq=%.2e λ=%.4f p=%d J=%.2e dt=%.1f µs",
            mp_.R, mp_.Ld, mp_.Lq, mp_.lambda, mp_.p, mp_.J_tot, dt_ * 1e6f);

        // ── Subscribers ────────────────────────────────────────────────────────
        auto qos = rclcpp::SystemDefaultsQoS();
        sub_vd_ = create_subscription<F64>("/foc/vd", qos,
            [this](const F64::SharedPtr msg){ vd_ = static_cast<float>(msg->data); });
        sub_vq_ = create_subscription<F64>("/foc/vq", qos,
            [this](const F64::SharedPtr msg){ vq_ = static_cast<float>(msg->data); });

        // ── Publishers ─────────────────────────────────────────────────────────
        auto lat = rclcpp::SensorDataQoS();
        pub_id_     = create_publisher<F64>("/foc/id",          lat);
        pub_iq_     = create_publisher<F64>("/foc/iq",          lat);
        pub_ialpha_ = create_publisher<F64>("/foc/i_alpha",     lat);
        pub_ibeta_  = create_publisher<F64>("/foc/i_beta",      lat);
        pub_theta_  = create_publisher<F64>("/foc/theta_real",  lat);
        pub_omega_  = create_publisher<F64>("/foc/omega",       lat);
        pub_torque_ = create_publisher<F64>("/foc/torque",      lat);

        // ── Timer ──────────────────────────────────────────────────────────────
        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(dt_ * 1e9));
        timer_ = create_wall_timer(period_ns,
            std::bind(&PmsmPlantNode::step, this));
    }

private:
    void step()
    {
        rk4_step(state_, vd_, vq_, mp_, dt_);

        // Convert dq → αβ for Phase 2 (observer uses αβ currents)
        const float c = std::cos(state_.theta_e);
        const float s = std::sin(state_.theta_e);
        // Inverse Park: i_alpha = cos·id − sin·iq,  i_beta = sin·id + cos·iq
        const float i_alpha = c * state_.id - s * state_.iq;
        const float i_beta  = s * state_.id + c * state_.iq;

        // Mechanical torque: τe = (3/2)·p·λ·Iq
        const float torque = 1.5f * static_cast<float>(mp_.p) * mp_.lambda * state_.iq;

        publish(pub_id_,     state_.id);
        publish(pub_iq_,     state_.iq);
        publish(pub_ialpha_, i_alpha);
        publish(pub_ibeta_,  i_beta);
        publish(pub_theta_,  state_.theta_e);
        publish(pub_omega_,  static_cast<float>(mp_.p) * state_.omega_m);  // electrical rad/s
        publish(pub_torque_, torque);
    }

    static void publish(rclcpp::Publisher<F64>::SharedPtr &pub, float val) {
        F64 msg;
        msg.data = static_cast<double>(val);
        pub->publish(msg);
    }

    // Parameters
    MotorParams mp_{};
    float dt_{1e-4f};

    // Input voltages (written by subscriber callbacks)
    float vd_{0.0f};
    float vq_{0.0f};

    // Integrator state
    State state_{};

    // ROS handles
    rclcpp::Subscription<F64>::SharedPtr sub_vd_, sub_vq_;
    rclcpp::Publisher<F64>::SharedPtr    pub_id_, pub_iq_,
                                         pub_ialpha_, pub_ibeta_,
                                         pub_theta_, pub_omega_, pub_torque_;
    rclcpp::TimerBase::SharedPtr         timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PmsmPlantNode>());
    rclcpp::shutdown();
    return 0;
}
