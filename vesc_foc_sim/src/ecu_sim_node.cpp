// ecu_sim_node.cpp
// Simulates the vehicle ECU: publishes iq_ref current setpoints and bus voltage.
//
// Profiles (set via 'profile' parameter):
//   "step"  — zero until t_start_s, then iq_ref_amplitude until t_stop_s, then zero
//   "ramp"  — linear ramp from 0 to iq_ref_amplitude over [t_start_s, t_stop_s]
//   "sine"  — sinusoidal at freq_hz with amplitude iq_ref_amplitude
//
// Published topics:
//   /iq_ref    [Float64]  current command [A]
//   /foc/vbus  [Float64]  bus voltage     [V]

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <string>

using F64 = std_msgs::msg::Float64;

class EcuSimNode : public rclcpp::Node
{
public:
    EcuSimNode() : Node("ecu_sim")
    {
        declare_parameter("iq_ref_amplitude", 50.0);
        declare_parameter("t_start_s",         0.5);
        declare_parameter("t_stop_s",          2.5);
        declare_parameter("vbus",              51.8);
        declare_parameter("profile",           std::string("step"));
        declare_parameter("freq_hz",           1.0);
        declare_parameter("publish_rate_hz",   1000.0);

        amp_        = static_cast<float>(get_parameter("iq_ref_amplitude").as_double());
        t_start_    = static_cast<float>(get_parameter("t_start_s").as_double());
        t_stop_     = static_cast<float>(get_parameter("t_stop_s").as_double());
        vbus_       = static_cast<float>(get_parameter("vbus").as_double());
        profile_    = get_parameter("profile").as_string();
        freq_hz_    = static_cast<float>(get_parameter("freq_hz").as_double());
        const double rate_hz = get_parameter("publish_rate_hz").as_double();

        RCLCPP_INFO(get_logger(),
            "ECU sim — profile='%s' amp=%.1fA t=[%.2f, %.2f]s Vbus=%.1fV @ %.0fHz",
            profile_.c_str(), amp_, t_start_, t_stop_, vbus_, rate_hz);

        auto qos = rclcpp::SensorDataQoS();
        pub_iq_   = create_publisher<F64>("/iq_ref",   qos);
        pub_vbus_ = create_publisher<F64>("/foc/vbus", qos);

        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1.0e9 / rate_hz));
        timer_ = create_wall_timer(period_ns,
            std::bind(&EcuSimNode::tick, this));

        t_now_ = 0.0f;
        dt_    = static_cast<float>(1.0 / rate_hz);
    }

private:
    void tick()
    {
        float iq_ref = compute_profile(t_now_);
        t_now_ += dt_;

        F64 msg_iq, msg_vbus;
        msg_iq.data   = static_cast<double>(iq_ref);
        msg_vbus.data = static_cast<double>(vbus_);

        pub_iq_->publish(msg_iq);
        pub_vbus_->publish(msg_vbus);
    }

    float compute_profile(float t) const {
        if (profile_ == "step") {
            if (t >= t_start_ && t < t_stop_) return amp_;
            return 0.0f;
        }
        if (profile_ == "ramp") {
            if (t < t_start_) return 0.0f;
            if (t > t_stop_)  return 0.0f;
            const float dur = t_stop_ - t_start_;
            if (dur <= 0.0f) return 0.0f;
            const float frac = (t - t_start_) / dur;
            // Ramp up then ramp down
            return amp_ * (1.0f - std::fabs(2.0f * frac - 1.0f));
        }
        if (profile_ == "sine") {
            if (t < t_start_) return 0.0f;
            return amp_ * std::sin(2.0f * static_cast<float>(M_PI) * freq_hz_ * (t - t_start_));
        }
        return 0.0f;
    }

    float amp_{50.0f};
    float t_start_{0.5f}, t_stop_{2.5f};
    float vbus_{51.8f};
    std::string profile_{"step"};
    float freq_hz_{1.0f};
    float t_now_{0.0f}, dt_{0.001f};

    rclcpp::Publisher<F64>::SharedPtr pub_iq_, pub_vbus_;
    rclcpp::TimerBase::SharedPtr      timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EcuSimNode>());
    rclcpp::shutdown();
    return 0;
}
