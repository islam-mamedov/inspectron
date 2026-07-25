#include "inspectron_motion_controller/motion_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "inspectron_mission_msgs/msg/mission_state.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspectron_motion_controller {

using inspectron::motion::MotionCommand;
using inspectron::motion::MotionLimits;
using inspectron::motion::PolicyMode;
using MissionState = inspectron_mission_msgs::msg::MissionState;
using PolicyDecision =
  inspectron_safety_supervisor::msg::PolicyDecision;
constexpr std::size_t kMaxRejectedFuturePolicyObservations = 256;

class MotionControllerNode final : public rclcpp::Node {
public:
  MotionControllerNode()
  : Node("motion_controller")
  {
    policy_timeout_ms_ =
      declare_parameter<std::int64_t>("policy_timeout_ms", 750);
    command_timeout_ms_ =
      declare_parameter<std::int64_t>("command_timeout_ms", 250);
    mission_state_timeout_ms_ =
      declare_parameter<std::int64_t>("mission_state_timeout_ms", 1000);
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

    rclcpp::QoS live_authority_qos(1);
    live_authority_qos.reliable();
    live_authority_qos.durability_volatile();

    policy_subscription_ = create_subscription<PolicyDecision>(
      "/inspectron/policy_decision",
      live_authority_qos,
      std::bind(
        &MotionControllerNode::on_policy_decision,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    mission_state_subscription_ = create_subscription<MissionState>(
      "/inspectron/mission/state",
      live_authority_qos,
      std::bind(
        &MotionControllerNode::on_mission_state,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    desired_command_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
        "/inspectron/desired_cmd_vel",
        10,
        std::bind(
          &MotionControllerNode::on_desired_command,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));

    control_timer_ = create_wall_timer(
      timer_period,
      std::bind(&MotionControllerNode::publish_safe_command, this));

    RCLCPP_INFO(
      get_logger(),
      "Fail-closed motion controller active: "
      "policy_timeout_ms=%ld command_timeout_ms=%ld "
      "mission_state_timeout_ms=%ld rate_hz=%.1f",
      static_cast<long>(policy_timeout_ms_),
      static_cast<long>(command_timeout_ms_),
      static_cast<long>(mission_state_timeout_ms_),
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

    if (mission_state_timeout_ms_ <= 0) {
      throw std::invalid_argument(
              "mission_state_timeout_ms must be positive");
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

  [[nodiscard]] std::optional<
    std::chrono::steady_clock::time_point>
  message_steady_reference(
    const rclcpp::MessageInfo & message_info,
    const std::int64_t timeout_ms,
    const builtin_interfaces::msg::Time * observation_timestamp =
    nullptr) const
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto ros_now_ns = get_clock()->now().nanoseconds();
    const auto system_now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::milliseconds(timeout_ms)).count();
    const auto & metadata = message_info.get_rmw_message_info();

    const auto timestamp_age =
      [timeout_ns](
      const std::int64_t now_ns,
      const std::int64_t timestamp_ns)
      -> std::optional<std::int64_t> {
        if (timestamp_ns <= 0 || timestamp_ns > now_ns) {
          return std::nullopt;
        }

        const auto age_ns = now_ns - timestamp_ns;
        if (age_ns > timeout_ns) {
          return std::nullopt;
        }

        return age_ns;
      };

    const auto source_age =
      timestamp_age(system_now_ns, metadata.source_timestamp);
    const auto received_age =
      timestamp_age(system_now_ns, metadata.received_timestamp);

    if (!source_age || !received_age) {
      return std::nullopt;
    }

    auto oldest_age = std::chrono::nanoseconds(
      *source_age >= *received_age ?
      *source_age : *received_age);
    if (observation_timestamp != nullptr) {
      if (observation_timestamp->sec < 0 ||
        observation_timestamp->nanosec >= 1'000'000'000U)
      {
        return std::nullopt;
      }

      const auto observation_ns =
        static_cast<std::int64_t>(observation_timestamp->sec) *
        1'000'000'000LL +
        static_cast<std::int64_t>(
        observation_timestamp->nanosec);
      const auto observation_age =
        timestamp_age(ros_now_ns, observation_ns);
      if (!observation_age) {
        return std::nullopt;
      }
      oldest_age = std::max(
        oldest_age,
        std::chrono::nanoseconds(*observation_age));
    }

    return steady_now - oldest_age;
  }

  void revoke_policy()
  {
    has_policy_ = false;
    policy_valid_ = false;
    policy_mode_ = PolicyMode::Stop;
    policy_evidence_id_.clear();
  }

  void reject_policy_temporally(
    const bool advance_recovery_barrier = true)
  {
    if (advance_recovery_barrier) {
      policy_recovery_barrier_ns_ =
        std::max(
        policy_recovery_barrier_ns_,
        get_clock()->now().nanoseconds());
    }
    revoke_policy();
  }

  void prune_rejected_future_policy_observations(
    const std::int64_t current_ns,
    const std::chrono::steady_clock::time_point steady_now)
  {
    bool opened_recovery_epoch = false;
    const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::milliseconds(policy_timeout_ms_)).count();
    const auto expired_before_ns = current_ns - timeout_ns;
    auto iterator =
      rejected_future_policy_observations_.begin();
    while (iterator !=
      rejected_future_policy_observations_.end())
    {
      if (*iterator < expired_before_ns ||
        *iterator <= latest_policy_observation_ns_)
      {
        iterator =
          rejected_future_policy_observations_.erase(iterator);
        opened_recovery_epoch = true;
      } else {
        ++iterator;
      }
    }
    if (rejected_future_policy_quarantine_until_ !=
      std::chrono::steady_clock::time_point{} &&
      steady_now > rejected_future_policy_quarantine_until_)
    {
      rejected_future_policy_quarantine_until_ = {};
      opened_recovery_epoch = true;
    }
    if (opened_recovery_epoch) {
      policy_recovery_barrier_ns_ =
        std::max(
        policy_recovery_barrier_ns_,
        current_ns);
    }
  }

  void remember_rejected_future_policy_observation(
    const std::int64_t observation_ns,
    const std::int64_t current_ns)
  {
    const auto steady_now =
      std::chrono::steady_clock::now();
    prune_rejected_future_policy_observations(
      current_ns,
      steady_now);
    if (rejected_future_policy_quarantine_until_ !=
      std::chrono::steady_clock::time_point{})
    {
      return;
    }

    rejected_future_policy_observations_.insert(observation_ns);
    if (rejected_future_policy_observations_.size() >
      kMaxRejectedFuturePolicyObservations)
    {
      rejected_future_policy_observations_.clear();
      rejected_future_policy_quarantine_until_ =
        steady_now +
        std::chrono::milliseconds(policy_timeout_ms_) * 2;
    }
  }

  void revoke_mission_authority()
  {
    has_mission_state_ = false;
    mission_state_ = MissionState::STATE_IDLE;
    motion_authorized_ = false;
    active_goal_.clear();
    mission_evidence_id_.clear();
  }

  void revoke_desired_command()
  {
    has_command_ = false;
    desired_command_ = {};
  }

  void on_policy_decision(
    const PolicyDecision::SharedPtr decision,
    const rclcpp::MessageInfo & message_info)
  {
    const auto cache_steady_now =
      std::chrono::steady_clock::now();
    const auto current_ns = get_clock()->now().nanoseconds();
    const bool observed_at_well_formed =
      decision->source_observed_at.sec >= 0 &&
      decision->source_observed_at.nanosec < 1'000'000'000U;
    std::int64_t observation_ns = 0;
    if (observed_at_well_formed) {
      observation_ns =
        static_cast<std::int64_t>(decision->source_observed_at.sec) *
        1'000'000'000LL +
        static_cast<std::int64_t>(
        decision->source_observed_at.nanosec);
    }

    prune_rejected_future_policy_observations(
      current_ns,
      cache_steady_now);
    const bool was_rejected_exactly_while_future =
      observed_at_well_formed &&
      rejected_future_policy_observations_.find(observation_ns) !=
      rejected_future_policy_observations_.end();
    const bool future_quarantine_active =
      rejected_future_policy_quarantine_until_ !=
      std::chrono::steady_clock::time_point{};
    if (future_quarantine_active &&
      observed_at_well_formed &&
      observation_ns > current_ns)
    {
      rejected_future_policy_quarantine_until_ =
        std::max(
        rejected_future_policy_quarantine_until_,
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(policy_timeout_ms_) * 2);
    }
    if (was_rejected_exactly_while_future ||
      future_quarantine_active)
    {
      reject_policy_temporally(
        /*advance_recovery_barrier=*/
        policy_recovery_barrier_ns_ == 0);
      return;
    }

    if (observed_at_well_formed && observation_ns > current_ns) {
      remember_rejected_future_policy_observation(
        observation_ns,
        current_ns);
      reject_policy_temporally();
      return;
    }

    const bool blocked_by_existing_barrier =
      observation_ns > 0 &&
      policy_recovery_barrier_ns_ > 0 &&
      observation_ns <= policy_recovery_barrier_ns_;
    if (blocked_by_existing_barrier) {
      reject_policy_temporally(
        /*advance_recovery_barrier=*/false);
      return;
    }

    const auto steady_reference =
      message_steady_reference(
      message_info,
      policy_timeout_ms_,
      &decision->source_observed_at);

    if (!steady_reference) {
      reject_policy_temporally();
      return;
    }

    if (observation_ns <= latest_policy_observation_ns_)
    {
      reject_policy_temporally();
      return;
    }

    latest_policy_observation_ns_ = observation_ns;
    policy_recovery_barrier_ns_ = 0;
    auto rejected_future =
      rejected_future_policy_observations_.begin();
    while (rejected_future !=
      rejected_future_policy_observations_.end() &&
      *rejected_future <= observation_ns)
    {
      rejected_future =
        rejected_future_policy_observations_.erase(
        rejected_future);
    }
    policy_received_at_ = *steady_reference;
    has_policy_ = true;
    policy_evidence_id_ = decision->evidence_id;
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

  void on_mission_state(
    const MissionState::SharedPtr state,
    const rclcpp::MessageInfo & message_info)
  {
    const auto steady_reference =
      message_steady_reference(
      message_info,
      mission_state_timeout_ms_);

    if (!steady_reference) {
      revoke_mission_authority();
      return;
    }

    mission_state_received_at_ = *steady_reference;
    has_mission_state_ = true;
    mission_state_ = state->state;
    motion_authorized_ = state->motion_authorized;
    active_goal_ = state->active_goal;
    mission_evidence_id_ = state->last_evidence_id;
  }

  void on_desired_command(
    const geometry_msgs::msg::Twist::SharedPtr command,
    const rclcpp::MessageInfo & message_info)
  {
    const auto steady_reference =
      message_steady_reference(message_info, command_timeout_ms_);

    if (!steady_reference) {
      revoke_desired_command();
      return;
    }

    desired_command_.linear_x = command->linear.x;
    desired_command_.angular_z = command->angular.z;
    command_received_at_ = *steady_reference;
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

    const bool mission_state_fresh =
      has_mission_state_ &&
      now - mission_state_received_at_ <=
      std::chrono::milliseconds(mission_state_timeout_ms_);

    const bool mission_authorized =
      mission_state_fresh &&
      mission_state_ == MissionState::STATE_MOVING &&
      motion_authorized_ &&
      !active_goal_.empty() &&
      !mission_evidence_id_.empty() &&
      mission_evidence_id_ == policy_evidence_id_;

    const MotionCommand safe_command =
      inspectron::motion::enforce_motion_policy(
      desired_command_,
      policy_mode_,
      policy_valid_ && mission_authorized,
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
  std::int64_t mission_state_timeout_ms_{1000};
  double control_rate_hz_{20.0};

  MotionLimits limits_{};
  MotionCommand desired_command_{};
  PolicyMode policy_mode_{PolicyMode::Stop};

  bool has_policy_{false};
  bool has_command_{false};
  bool has_mission_state_{false};
  bool policy_valid_{false};
  bool motion_authorized_{false};

  std::uint8_t mission_state_{MissionState::STATE_IDLE};
  std::string active_goal_{};
  std::string mission_evidence_id_{};
  std::string policy_evidence_id_{};
  std::int64_t latest_policy_observation_ns_{0};
  std::int64_t policy_recovery_barrier_ns_{0};
  std::set<std::int64_t>
    rejected_future_policy_observations_;
  std::chrono::steady_clock::time_point
    rejected_future_policy_quarantine_until_{};

  std::chrono::steady_clock::time_point policy_received_at_{};
  std::chrono::steady_clock::time_point command_received_at_{};
  std::chrono::steady_clock::time_point mission_state_received_at_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_publisher_;

  rclcpp::Subscription<PolicyDecision>::SharedPtr
    policy_subscription_;

  rclcpp::Subscription<MissionState>::SharedPtr
    mission_state_subscription_;

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
