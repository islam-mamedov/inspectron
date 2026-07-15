#include "inspectron_motion_controller/motion_policy.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspectron_motion_controller {

using inspectron::motion::MotionCommand;
using inspectron::motion::MotionLimits;
using inspectron::motion::PolicyMode;
using PolicyDecision =
  inspectron_safety_supervisor::msg::PolicyDecision;

class MotionControllerNode final : public rclcpp::Node {
public:
  MotionControllerNode()
  : Node("motion_controller")
  {
    policy_timeout_ms_ =
      declare_parameter<std::int64_t>("policy_timeout_ms", 750);
    command_timeout_ms_ =
      declare_parameter<std::int64_t>("command_timeout_ms", 250);
    control_rate_hz_ =
      declare_parameter<double>("control_rate_hz", 20.0);

    limits_.slow_scale =
      declare_parameter<double>("slow_scale", 0.35);
    limits_.max_linear_speed =
      declare_parameter<double>("max_linear_speed", 0.40);
    limits_.max_angular_speed =
      declare_parameter<double>("max_angular_speed", 0.80);

    validate_parameters();

    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    rclcpp::QoS policy_qos(1);
    policy_qos.reliable();
    policy_qos.transient_local();

    policy_subscription_ = create_subscription<PolicyDecision>(
      "/inspectron/policy_decision",
      policy_qos,
      std::bind(
        &MotionControllerNode::on_policy_decision,
        this,
        std::placeholders::_1));

    desired_command_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
        "/inspectron/desired_cmd_vel",
        10,
        std::bind(
          &MotionControllerNode::on_desired_command,
          this,
          std::placeholders::_1));

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));

    control_timer_ = create_wall_timer(
      timer_period,
      std::bind(&MotionControllerNode::publish_safe_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Fail-closed motion controller active: "
      "policy_timeout_ms=%ld command_timeout_ms=%ld rate_hz=%.1f",
      static_cast<long>(policy_timeout_ms_),
      static_cast<long>(command_timeout_ms_),
      control_rate_hz_);
  }

private:
  void validate_parameters() const
  {
    if (policy_timeout_ms_ <= 0) {
      throw std::invalid_argument(
              "policy_timeout_ms must be positive");
    }

    if (command_timeout_ms_ <= 0) {
      throw std::invalid_argument(
              "command_timeout_ms must be positive");
    }

    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
      throw std::invalid_argument(
              "control_rate_hz must be finite and positive");
    }

    if (!std::isfinite(limits_.max_linear_speed) ||
        limits_.max_linear_speed <= 0.0) {
      throw std::invalid_argument(
              "max_linear_speed must be finite and positive");
    }

    if (!std::isfinite(limits_.max_angular_speed) ||
        limits_.max_angular_speed <= 0.0) {
      throw std::invalid_argument(
              "max_angular_speed must be finite and positive");
    }

    if (!std::isfinite(limits_.slow_scale) ||
        limits_.slow_scale < 0.0 ||
        limits_.slow_scale > 1.0) {
      throw std::invalid_argument(
              "slow_scale must be between zero and one");
    }
  }

  void on_policy_decision(
    const PolicyDecision::SharedPtr decision)
  {
    policy_received_at_ = std::chrono::steady_clock::now();
    has_policy_ = true;
    policy_valid_ =
      decision->status == PolicyDecision::STATUS_VALID;

    if (!policy_valid_) {
      policy_mode_ = PolicyMode::Stop;
      return;
    }

    switch (decision->action) {
      case PolicyDecision::ACTION_PROCEED:
        policy_mode_ = PolicyMode::Proceed;
        break;

      case PolicyDecision::ACTION_SLOW_DOWN:
        policy_mode_ = PolicyMode::Slow;
        break;

      case PolicyDecision::ACTION_STOP:
      default:
        policy_mode_ = PolicyMode::Stop;
        break;
    }
  }

  void on_desired_command(
    const geometry_msgs::msg::Twist::SharedPtr command)
  {
    desired_command_.linear_x = command->linear.x;
    desired_command_.angular_z = command->angular.z;
    command_received_at_ = std::chrono::steady_clock::now();
    has_command_ = true;
  }

  void publish_safe_command()
  {
    const auto now = std::chrono::steady_clock::now();

    const bool policy_fresh =
      has_policy_ &&
      now - policy_received_at_ <=
      std::chrono::milliseconds(policy_timeout_ms_);

    const bool command_fresh =
      has_command_ &&
      now - command_received_at_ <=
      std::chrono::milliseconds(command_timeout_ms_);

    const MotionCommand safe_command =
      inspectron::motion::enforce_motion_policy(
      desired_command_,
      policy_mode_,
      policy_valid_,
      policy_fresh,
      command_fresh,
      limits_);

    geometry_msgs::msg::Twist output;
    output.linear.x = safe_command.linear_x;
    output.angular.z = safe_command.angular_z;
    command_publisher_->publish(output);
  }

  std::int64_t policy_timeout_ms_{750};
  std::int64_t command_timeout_ms_{250};
  double control_rate_hz_{20.0};

  MotionLimits limits_{};
  MotionCommand desired_command_{};
  PolicyMode policy_mode_{PolicyMode::Stop};

  bool has_policy_{false};
  bool has_command_{false};
  bool policy_valid_{false};

  std::chrono::steady_clock::time_point policy_received_at_{};
  std::chrono::steady_clock::time_point command_received_at_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_publisher_;

  rclcpp::Subscription<PolicyDecision>::SharedPtr
    policy_subscription_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    desired_command_subscription_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace inspectron_motion_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      inspectron_motion_controller::MotionControllerNode>());

  rclcpp::shutdown();
  return 0;
}
