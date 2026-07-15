#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "inspectron/safety_policy.hpp"
#include "inspectron_safety_supervisor/message_conversion.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "inspectron_safety_supervisor/msg/scene_assessment.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspectron_safety_supervisor {
namespace {

using namespace std::chrono_literals;
using SceneMessage = msg::SceneAssessment;
using DecisionMessage = msg::PolicyDecision;

[[nodiscard]] bool is_probability(double value) noexcept {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

class SafetySupervisorNode final : public rclcpp::Node {
 public:
  SafetySupervisorNode()
      : rclcpp::Node("safety_supervisor"),
        started_at_(std::chrono::steady_clock::now()) {
    const auto input_topic = declare_parameter<std::string>(
        "input_topic", "/inspectron/scene_assessment");
    const auto decision_topic = declare_parameter<std::string>(
        "decision_topic", "/inspectron/policy_decision");

    thresholds_.confidence =
        declare_parameter<double>("confidence_threshold", 0.70);
    thresholds_.view_quality =
        declare_parameter<double>("view_quality_threshold", 0.60);

    const auto timeout_ms =
        declare_parameter<std::int64_t>("assessment_timeout_ms", 1500);

    if (input_topic.empty() || decision_topic.empty()) {
      throw std::invalid_argument("ROS topic names must not be empty");
    }
    if (!is_probability(thresholds_.confidence) ||
        !is_probability(thresholds_.view_quality)) {
      throw std::invalid_argument("policy thresholds must be within [0, 1]");
    }
    if (timeout_ms <= 0) {
      throw std::invalid_argument("assessment_timeout_ms must be positive");
    }

    assessment_timeout_ = std::chrono::milliseconds{timeout_ms};

    auto decision_qos = rclcpp::QoS{rclcpp::KeepLast{1}};
    decision_qos.reliable().transient_local();
    decision_publisher_ =
        create_publisher<DecisionMessage>(decision_topic, decision_qos);

    auto assessment_qos = rclcpp::QoS{rclcpp::KeepLast{10}};
    assessment_qos.reliable();
    assessment_subscription_ = create_subscription<SceneMessage>(
        input_topic,
        assessment_qos,
        std::bind(
            &SafetySupervisorNode::handle_assessment,
            this,
            std::placeholders::_1));

    const auto timer_period = std::max(assessment_timeout_ / 4, 10ms);
    watchdog_timer_ = create_wall_timer(
        timer_period,
        std::bind(&SafetySupervisorNode::check_watchdog, this));

    RCLCPP_INFO(
        get_logger(),
        "Safety supervisor active: input=%s output=%s timeout_ms=%lld",
        input_topic.c_str(),
        decision_topic.c_str(),
        static_cast<long long>(timeout_ms));
  }

 private:
  void handle_assessment(const SceneMessage::SharedPtr input) {
    {
      std::scoped_lock lock{state_mutex_};
      received_assessment_ = true;
      timeout_published_ = false;
      last_assessment_at_ = std::chrono::steady_clock::now();
      last_evidence_id_ = input->evidence_id;
    }

    const auto assessment = from_message(*input);
    const bool input_is_valid =
        inspectron::is_valid_assessment(assessment, thresholds_);
    const auto policy_decision =
        inspectron::resolve_policy(assessment, thresholds_);

    const auto status = input_is_valid ? DecisionMessage::STATUS_VALID
                                       : DecisionMessage::STATUS_INVALID;
    auto output = to_message(policy_decision, status, input->evidence_id);
    output.source_observed_at = input->observed_at;
    output.decided_at = now();
    decision_publisher_->publish(output);

    if (!input_is_valid) {
      RCLCPP_ERROR(
          get_logger(),
          "Rejected invalid assessment evidence_id=%s; enforced stop",
          input->evidence_id.c_str());
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Decision evidence_id=%s action=%s reason=%s overridden=%s",
        input->evidence_id.c_str(),
        inspectron::to_string(policy_decision.action).data(),
        inspectron::to_string(policy_decision.reason).data(),
        policy_decision.model_action_overridden ? "true" : "false");
  }

  void check_watchdog() {
    std::string evidence_id;

    {
      std::scoped_lock lock{state_mutex_};
      const auto now_steady = std::chrono::steady_clock::now();
      const auto reference =
          received_assessment_ ? last_assessment_at_ : started_at_;

      if (timeout_published_ ||
          now_steady - reference < assessment_timeout_) {
        return;
      }

      timeout_published_ = true;
      evidence_id = last_evidence_id_;
    }

    const inspectron::PolicyDecision stop_decision{
        .action = inspectron::RecommendedAction::Stop,
        .reason = inspectron::DecisionReason::InvalidAssessment,
        .model_action_overridden = false,
    };

    auto output = to_message(
        stop_decision, DecisionMessage::STATUS_STALE, evidence_id);
    output.decided_at = now();
    decision_publisher_->publish(output);

    RCLCPP_ERROR(
        get_logger(),
        "Assessment watchdog expired after %lld ms; enforced stop",
        static_cast<long long>(assessment_timeout_.count()));
  }

  inspectron::PolicyThresholds thresholds_{};
  std::chrono::milliseconds assessment_timeout_{1500};
  const std::chrono::steady_clock::time_point started_at_;

  std::mutex state_mutex_;
  bool received_assessment_{false};
  bool timeout_published_{false};
  std::chrono::steady_clock::time_point last_assessment_at_{};
  std::string last_evidence_id_{};

  rclcpp::Publisher<DecisionMessage>::SharedPtr decision_publisher_;
  rclcpp::Subscription<SceneMessage>::SharedPtr assessment_subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace
}  // namespace inspectron_safety_supervisor

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
        std::make_shared<
            inspectron_safety_supervisor::SafetySupervisorNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(
        rclcpp::get_logger("inspectron_safety_supervisor"),
        "Safety supervisor terminated: %s",
        error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
