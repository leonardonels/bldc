// pmsm_plant_node.cpp
// PMSM physical plant: Euler integration of dq ODE, ported from motor/virtual_motor.c.
//
// Differences from the original virtual_motor.c:
//   - Input is /foc/vd + /foc/vq (dq frame) rather than v_alpha/v_beta — no input
//     Park transform needed.
//   - theta_e is tracked as a true electrical angle (theta_e += p*omega_m*dt) which
//     corrects a latent multi-pole bug in virtual_motor.c where phi accumulated as a
//     mechanical angle but was used in Park transforms expecting an electrical angle.
//   - Viscous damping (B term) is omitted, matching virtual_motor.c.
//   - Reluctance torque term is included: τe = (3/2)·p·(λ + (Ld−Lq)·Id)·Iq
//
// ODE (rotor reference frame, Euler):
//   id_int  += (Vd − R·id + ωe·Lq·iq) · dt / Ld
//   id       = id_int − λ/Ld
//   diq/dt   = (Vq − R·iq − ωe·(Ld·id + λ)) / Lq
//   τe       = (3/2)·p·(λ + (Ld−Lq)·id)·iq
//   dωm/dt   = (τe − τload) / J
//   dθe/dt   = p · ωm
//
// Subscribed topics:
//   /foc/vd        [Float64]  d-axis voltage from controller [V]
//   /foc/vq        [Float64]  q-axis voltage from controller [V]
//
// Published topics:
//   /foc/id        [Float64]  d-axis current  [A]
//   /foc/iq        [Float64]  q-axis current  [A]
//   /foc/i_alpha   [Float64]  α-frame current [A]
//   /foc/i_beta    [Float64]  β-frame current [A]
//   /foc/theta_real[Float64]  electrical angle [rad]
//   /foc/omega     [Float64]  electrical speed [rad/s]
//   /foc/torque    [Float64]  electromagnetic torque [Nm]

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "vesc_foc_sim/adapter.hpp"

using F64 = std_msgs::msg::Float64;

class PmsmPlantNode : public rclcpp::Node
{
public:
    PmsmPlantNode() : Node("pmsm_plant")
    {
        declare_parameter("R",        0.03);
        declare_parameter("Ld",       7.0e-5);
        declare_parameter("Lq",       1.1e-4);
        declare_parameter("lambda",   0.0115);
        declare_parameter("p",        4);
        declare_parameter("J_tot",    7.5e-4);
        declare_parameter("tau_load", 0.0);
        declare_parameter("sim_dt_s", 1.0e-4);

        R_        = static_cast<float>(get_parameter("R").as_double());
        Ld_       = static_cast<float>(get_parameter("Ld").as_double());
        Lq_       = static_cast<float>(get_parameter("Lq").as_double());
        lambda_   = static_cast<float>(get_parameter("lambda").as_double());
        p_        = get_parameter("p").as_int();
        J_        = static_cast<float>(get_parameter("J_tot").as_double());
        tau_load_ = static_cast<float>(get_parameter("tau_load").as_double());
        dt_       = static_cast<float>(get_parameter("sim_dt_s").as_double());

        RCLCPP_INFO(get_logger(),
            "Plant (virtual_motor) — R=%.4f Ld=%.2e Lq=%.2e λ=%.4f p=%d J=%.2e dt=%.1f µs",
            R_, Ld_, Lq_, lambda_, p_, J_, dt_ * 1e6f);

        // Initialise id integral state so id starts at 0:  id = id_int - λ/Ld → id_int = λ/Ld
        id_int_ = lambda_ / Ld_;

        auto qos = rclcpp::SystemDefaultsQoS();
        sub_vd_ = create_subscription<F64>("/foc/vd", qos,
            [this](const F64::SharedPtr m){ vd_ = static_cast<float>(m->data); });
        sub_vq_ = create_subscription<F64>("/foc/vq", qos,
            [this](const F64::SharedPtr m){ vq_ = static_cast<float>(m->data); });

        auto lat = rclcpp::SensorDataQoS();
        pub_id_     = create_publisher<F64>("/foc/id",         lat);
        pub_iq_     = create_publisher<F64>("/foc/iq",         lat);
        pub_ialpha_ = create_publisher<F64>("/foc/i_alpha",    lat);
        pub_ibeta_  = create_publisher<F64>("/foc/i_beta",     lat);
        pub_theta_  = create_publisher<F64>("/foc/theta_real", lat);
        pub_omega_  = create_publisher<F64>("/foc/omega",      lat);
        pub_torque_ = create_publisher<F64>("/foc/torque",     lat);

        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(dt_ * 1e9));
        timer_ = create_wall_timer(period_ns,
            std::bind(&PmsmPlantNode::step, this));
    }

private:
    void step()
    {
        float i_alpha, i_beta, torque;
        foc_math::vm_step(
            id_int_, id_, iq_, omega_m_, theta_e_,
            vd_, vq_, dt_,
            R_, Ld_, Lq_, lambda_, J_, tau_load_, p_,
            i_alpha, i_beta, torque);

        pub(pub_id_,     id_);
        pub(pub_iq_,     iq_);
        pub(pub_ialpha_, i_alpha);
        pub(pub_ibeta_,  i_beta);
        pub(pub_theta_,  theta_e_);
        pub(pub_omega_,  static_cast<float>(p_) * omega_m_);
        pub(pub_torque_, torque);
    }

    static void pub(rclcpp::Publisher<F64>::SharedPtr &p, float v) {
        F64 m; m.data = static_cast<double>(v); p->publish(m);
    }

    // Motor params
    float R_{}, Ld_{}, Lq_{}, lambda_{};
    int   p_{4};
    float J_{}, tau_load_{};
    float dt_{1e-4f};

    // Inputs
    float vd_{0.0f}, vq_{0.0f};

    // Plant state
    float id_int_{0.0f};   // d-axis current integral state
    float id_{0.0f};
    float iq_{0.0f};
    float omega_m_{0.0f};  // mechanical speed [rad/s]
    float theta_e_{0.0f};  // electrical angle [rad]

    rclcpp::Subscription<F64>::SharedPtr sub_vd_, sub_vq_;
    rclcpp::Publisher<F64>::SharedPtr    pub_id_, pub_iq_,
                                         pub_ialpha_, pub_ibeta_,
                                         pub_theta_, pub_omega_, pub_torque_;
    rclcpp::TimerBase::SharedPtr         timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PmsmPlantNode>());
    rclcpp::shutdown();
    return 0;
}
