// ecu_sim_node.cpp
// Simulates the vehicle ECU: publishes iq_ref current setpoints and bus voltage.
//
// Profiles (set via 'profile' parameter):
//   "step"          — zero until t_start_s, then iq_ref_amplitude until t_stop_s, then zero
//   "ramp"          — linear ramp from 0 to iq_ref_amplitude over [t_start_s, t_stop_s]
//   "sine"          — sinusoidal at freq_hz with amplitude iq_ref_amplitude
//   "speed_profile" — closed-loop outer PI speed loop tracking YAML-defined waypoints;
//                     reads /foc/omega, outputs iq_ref clamped to l_current_max
//
// Published topics:
//   /iq_ref    [Float64]  current command [A]
//   /foc/vbus  [Float64]  bus voltage     [V]
//
// Subscribed topics (speed_profile only):
//   /foc/omega [Float64]  measured rotor speed [rad/s]

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <string>
#include <vector>

using F64 = std_msgs::msg::Float64;

struct Waypoint {
    float t_s;
    float omega_target_radps;
};

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

        // Speed-profile PI parameters
        declare_parameter("kp_speed",       0.5);
        declare_parameter("ki_speed",       2.0);
        declare_parameter("l_current_max", 110.0);

        // Speed waypoints: list of {t_s, omega_target_radps}
        declare_parameter("speed_waypoints.t_s",                std::vector<double>{});
        declare_parameter("speed_waypoints.omega_target_radps", std::vector<double>{});

        amp_        = static_cast<float>(get_parameter("iq_ref_amplitude").as_double());
        t_start_    = static_cast<float>(get_parameter("t_start_s").as_double());
        t_stop_     = static_cast<float>(get_parameter("t_stop_s").as_double());
        vbus_       = static_cast<float>(get_parameter("vbus").as_double());
        profile_    = get_parameter("profile").as_string();
        freq_hz_    = static_cast<float>(get_parameter("freq_hz").as_double());
        kp_speed_   = static_cast<float>(get_parameter("kp_speed").as_double());
        ki_speed_   = static_cast<float>(get_parameter("ki_speed").as_double());
        i_max_      = static_cast<float>(get_parameter("l_current_max").as_double());

        const double rate_hz = get_parameter("publish_rate_hz").as_double();
        dt_ = static_cast<float>(1.0 / rate_hz);

        if (profile_ == "speed_profile") {
            load_waypoints();
            RCLCPP_INFO(get_logger(),
                "ECU sim — profile='speed_profile' kp=%.3f ki=%.3f i_max=%.1fA "
                "waypoints=%zu Vbus=%.1fV @ %.0fHz",
                kp_speed_, ki_speed_, i_max_, waypoints_.size(), vbus_, rate_hz);
        } else {
            RCLCPP_INFO(get_logger(),
                "ECU sim — profile='%s' amp=%.1fA t=[%.2f, %.2f]s Vbus=%.1fV @ %.0fHz",
                profile_.c_str(), amp_, t_start_, t_stop_, vbus_, rate_hz);
        }

        auto qos = rclcpp::SensorDataQoS();
        pub_iq_   = create_publisher<F64>("/iq_ref",   qos);
        pub_vbus_ = create_publisher<F64>("/foc/vbus", qos);

        if (profile_ == "speed_profile") {
            sub_omega_ = create_subscription<F64>(
                "/foc/omega", qos,
                [this](const F64::SharedPtr msg) {
                    omega_meas_     = static_cast<float>(msg->data);
                    omega_received_ = true;
                });
        }

        const auto period_ns = std::chrono::nanoseconds(
            static_cast<int64_t>(1.0e9 / rate_hz));
        timer_ = create_wall_timer(period_ns,
            std::bind(&EcuSimNode::tick, this));

        t_now_ = 0.0f;
    }

private:
    void load_waypoints()
    {
        const auto t_vec   = get_parameter("speed_waypoints.t_s").as_double_array();
        const auto w_vec   = get_parameter("speed_waypoints.omega_target_radps").as_double_array();

        if (t_vec.size() != w_vec.size()) {
            RCLCPP_WARN(get_logger(),
                "speed_waypoints.t_s (%zu) and omega_target_radps (%zu) differ in length — "
                "using minimum", t_vec.size(), w_vec.size());
        }
        const size_t n = std::min(t_vec.size(), w_vec.size());
        waypoints_.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            waypoints_.push_back({static_cast<float>(t_vec[i]),
                                  static_cast<float>(w_vec[i])});
        }
    }

    float active_omega_target() const
    {
        if (waypoints_.empty()) return 0.0f;

        // Walk backwards: first waypoint whose t_s <= t_now_
        for (int i = static_cast<int>(waypoints_.size()) - 1; i >= 0; --i) {
            if (t_now_ >= waypoints_[i].t_s) {
                return waypoints_[i].omega_target_radps;
            }
        }
        // Before first waypoint
        return 0.0f;
    }

    void tick()
    {
        float iq_ref = 0.0f;

        if (profile_ == "speed_profile") {
            if (!omega_received_) {
                // Freeze integrator until first measurement arrives
                iq_ref = 0.0f;
            } else {
                const float target = active_omega_target();
                const float error  = target - omega_meas_;

                // PI with anti-windup (back-calculation / clamp)
                integral_ += ki_speed_ * error * dt_;
                float output = kp_speed_ * error + integral_;

                // Clamp output and apply anti-windup to integral
                const float clamped = std::clamp(output, -i_max_, i_max_);
                if (output != clamped) {
                    integral_ -= output - clamped;
                }
                iq_ref = clamped;
            }
        } else {
            iq_ref = compute_open_loop(t_now_);
        }

        t_now_ += dt_;

        F64 msg_iq, msg_vbus;
        msg_iq.data   = static_cast<double>(iq_ref);
        msg_vbus.data = static_cast<double>(vbus_);
        pub_iq_->publish(msg_iq);
        pub_vbus_->publish(msg_vbus);
    }

    float compute_open_loop(float t) const
    {
        if (profile_ == "step") {
            if (t >= t_start_ && t < t_stop_) return amp_;
            return 0.0f;
        }
        if (profile_ == "ramp") {
            if (t < t_start_ || t > t_stop_) return 0.0f;
            const float dur = t_stop_ - t_start_;
            if (dur <= 0.0f) return 0.0f;
            const float frac = (t - t_start_) / dur;
            return amp_ * (1.0f - std::fabs(2.0f * frac - 1.0f));
        }
        if (profile_ == "sine") {
            if (t < t_start_) return 0.0f;
            return amp_ * std::sin(2.0f * static_cast<float>(M_PI) * freq_hz_ * (t - t_start_));
        }
        return 0.0f;
    }

    // Open-loop profile parameters
    float amp_{50.0f};
    float t_start_{0.5f}, t_stop_{2.5f};
    float freq_hz_{1.0f};

    // Speed-profile PI state
    float kp_speed_{0.5f}, ki_speed_{2.0f}, i_max_{110.0f};
    float integral_{0.0f};
    float omega_meas_{0.0f};
    bool  omega_received_{false};
    std::vector<Waypoint> waypoints_;

    // Common
    float vbus_{51.8f};
    std::string profile_{"step"};
    float t_now_{0.0f}, dt_{0.001f};

    rclcpp::Publisher<F64>::SharedPtr    pub_iq_, pub_vbus_;
    rclcpp::Subscription<F64>::SharedPtr sub_omega_;
    rclcpp::TimerBase::SharedPtr         timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EcuSimNode>());
    rclcpp::shutdown();
    return 0;
}
