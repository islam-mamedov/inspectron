#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "inspectron/safety_policy.hpp"
#include "inspectron_safety_supervisor/message_conversion.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "inspectron_safety_supervisor/msg/scene_assessment.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inspectron_safety_supervisor {
namespace {

using namespace std::chrono_literals;
using SceneMessage = msg::SceneAssessment;
using DecisionMessage = msg::PolicyDecision;
constexpr std::size_t kMaxRejectedFutureObservations = 256;

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
            std::placeholders::_1,
            std::placeholders::_2));

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
  [[nodiscard]] std::optional<std::chrono::steady_clock::time_point>
  assessment_steady_reference(
      const SceneMessage& input,
      const rclcpp::MessageInfo& message_info) const {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto observation_now_ns = now().nanoseconds();
    const auto metadata_now_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const auto timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            assessment_timeout_)
            .count();
    const auto& metadata = message_info.get_rmw_message_info();

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

    if (input.observed_at.sec < 0 ||
        input.observed_at.nanosec >= 1'000'000'000U) {
      return std::nullopt;
    }

    const auto observed_at_ns =
        static_cast<std::int64_t>(input.observed_at.sec) *
            1'000'000'000LL +
        static_cast<std::int64_t>(input.observed_at.nanosec);
    const auto observation_age =
        timestamp_age(observation_now_ns, observed_at_ns);
    const auto source_age =
        timestamp_age(metadata_now_ns, metadata.source_timestamp);
    const auto received_age =
        timestamp_age(metadata_now_ns, metadata.received_timestamp);

    if (!observation_age || !source_age || !received_age) {
      return std::nullopt;
    }

    const auto oldest_age = std::max(
        {*observation_age, *source_age, *received_age});
    return steady_now - std::chrono::nanoseconds{oldest_age};
  }

  [[nodiscard]] static bool observation_is_strictly_newer(
      const builtin_interfaces::msg::Time& candidate,
      const builtin_interfaces::msg::Time& previous) {
    return candidate.sec > previous.sec ||
           (candidate.sec == previous.sec &&
            candidate.nanosec > previous.nanosec);
  }

  void prune_rejected_future_observations_locked(
      const std::int64_t current_ns,
      const std::chrono::steady_clock::time_point steady_now) {
    const auto timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            assessment_timeout_)
            .count();
    const auto expired_before_ns = current_ns - timeout_ns;
    bool opened_recovery_epoch = false;
    auto iterator = rejected_future_observations_.begin();
    while (iterator != rejected_future_observations_.end()) {
      if (*iterator < expired_before_ns ||
          (received_assessment_ &&
           *iterator <=
               static_cast<std::int64_t>(
                   last_observed_at_.sec) *
                       1'000'000'000LL +
                   static_cast<std::int64_t>(
                       last_observed_at_.nanosec))) {
        iterator = rejected_future_observations_.erase(iterator);
        opened_recovery_epoch = true;
      } else {
        ++iterator;
      }
    }
    if (rejected_future_quarantine_until_ !=
            std::chrono::steady_clock::time_point{} &&
        steady_now > rejected_future_quarantine_until_) {
      rejected_future_quarantine_until_ = {};
      opened_recovery_epoch = true;
    }
    if (opened_recovery_epoch) {
      recovery_barrier_ns_ =
          std::max(recovery_barrier_ns_, current_ns);
    }
  }

  void remember_rejected_future_observation_locked(
      const std::int64_t observation_ns,
      const std::int64_t current_ns) {
    const auto steady_now =
        std::chrono::steady_clock::now();
    prune_rejected_future_observations_locked(
        current_ns,
        steady_now);
    if (rejected_future_quarantine_until_ !=
        std::chrono::steady_clock::time_point{}) {
      return;
    }

    rejected_future_observations_.insert(observation_ns);
    if (rejected_future_observations_.size() >
        kMaxRejectedFutureObservations) {
      rejected_future_observations_.clear();
      rejected_future_quarantine_until_ =
          steady_now + assessment_timeout_ * 2;
    }
  }

  void publish_stale_assessment_stop(
      const SceneMessage& input,
      const bool advance_recovery_barrier = true) {
    const auto rejected_at = now();
    {
      std::scoped_lock lock{state_mutex_};
      timeout_published_ = true;
      if (advance_recovery_barrier) {
        const auto rejected_at_ns = rejected_at.nanoseconds();
        if (input.observed_at.sec >= 0 &&
            input.observed_at.nanosec < 1'000'000'000U) {
          const auto rejected_observation_ns =
              static_cast<std::int64_t>(input.observed_at.sec) *
                  1'000'000'000LL +
              static_cast<std::int64_t>(input.observed_at.nanosec);
          if (rejected_observation_ns > rejected_at_ns) {
            remember_rejected_future_observation_locked(
                rejected_observation_ns,
                rejected_at_ns);
          }
        }
        recovery_barrier_ns_ =
            std::max(recovery_barrier_ns_, rejected_at_ns);
      }
    }

    const inspectron::PolicyDecision stop_decision{
        .action = inspectron::RecommendedAction::Stop,
        .reason = inspectron::DecisionReason::InvalidAssessment,
        .model_action_overridden = false,
    };

    auto output = to_message(
        stop_decision,
        DecisionMessage::STATUS_STALE,
        input.evidence_id);
    output.source_observed_at = input.observed_at;
    output.decided_at = rejected_at;
    decision_publisher_->publish(output);

    RCLCPP_ERROR(
        get_logger(),
        "Rejected assessment with missing, future, expired, or "
        "non-monotonic temporal metadata evidence_id=%s; enforced stop",
        input.evidence_id.c_str());
  }

  void handle_assessment(
      const SceneMessage::SharedPtr input,
      const rclcpp::MessageInfo& message_info) {
    const bool observed_at_well_formed =
        input->observed_at.sec >= 0 &&
        input->observed_at.nanosec < 1'000'000'000U;
    const auto observed_at_ns =
        observed_at_well_formed
            ? static_cast<std::int64_t>(input->observed_at.sec) *
                      1'000'000'000LL +
                  static_cast<std::int64_t>(
                      input->observed_at.nanosec)
            : 0;
    if (observed_at_well_formed) {
      const auto steady_check_now =
          std::chrono::steady_clock::now();
      const auto observation_check_now_ns =
          now().nanoseconds();
      bool was_rejected_exactly_while_future = false;
      bool future_quarantine_active = false;
      bool advance_recovery_barrier = true;
      {
        std::scoped_lock lock{state_mutex_};
        prune_rejected_future_observations_locked(
            observation_check_now_ns,
            steady_check_now);
        was_rejected_exactly_while_future =
            rejected_future_observations_.find(observed_at_ns) !=
            rejected_future_observations_.end();
        future_quarantine_active =
            rejected_future_quarantine_until_ !=
            std::chrono::steady_clock::time_point{};
        if (future_quarantine_active &&
            observed_at_ns > observation_check_now_ns) {
          rejected_future_quarantine_until_ = std::max(
              rejected_future_quarantine_until_,
              std::chrono::steady_clock::now() +
                  assessment_timeout_ * 2);
        }
        advance_recovery_barrier =
            recovery_barrier_ns_ == 0;
      }
      if (was_rejected_exactly_while_future ||
          future_quarantine_active) {
        publish_stale_assessment_stop(
            *input,
            advance_recovery_barrier);
        return;
      }
    }

    if (observed_at_ns > 0) {
      bool blocked_by_existing_barrier = false;
      {
        std::scoped_lock lock{state_mutex_};
        blocked_by_existing_barrier =
            recovery_barrier_ns_ > 0 &&
            observed_at_ns <= recovery_barrier_ns_;
      }
      if (blocked_by_existing_barrier) {
        publish_stale_assessment_stop(
            *input,
            /*advance_recovery_barrier=*/false);
        return;
      }
    }

    const auto steady_reference =
        assessment_steady_reference(*input, message_info);

    if (!steady_reference) {
      publish_stale_assessment_stop(*input);
      return;
    }

    bool observation_admitted = false;
    bool advance_recovery_barrier = true;
    {
      std::scoped_lock lock{state_mutex_};
      const bool advances_last_observation =
          !received_assessment_ ||
          observation_is_strictly_newer(
              input->observed_at, last_observed_at_);
      const bool blocked_by_existing_barrier =
          recovery_barrier_ns_ > 0 &&
          observed_at_ns <= recovery_barrier_ns_;
      if (advances_last_observation &&
          !blocked_by_existing_barrier) {
        observation_admitted = true;
        received_assessment_ = true;
        timeout_published_ = false;
        recovery_barrier_ns_ = 0;
        last_assessment_at_ = *steady_reference;
        last_evidence_id_ = input->evidence_id;
        last_observed_at_ = input->observed_at;
        auto rejected_future =
            rejected_future_observations_.begin();
        while (rejected_future !=
                   rejected_future_observations_.end() &&
               *rejected_future <= observed_at_ns) {
          rejected_future =
              rejected_future_observations_.erase(rejected_future);
        }
      } else if (blocked_by_existing_barrier) {
        advance_recovery_barrier = false;
      }
    }

    if (!observation_admitted) {
      publish_stale_assessment_stop(
          *input,
          advance_recovery_barrier);
      return;
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
    builtin_interfaces::msg::Time source_observed_at;

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
      source_observed_at = last_observed_at_;
    }

    const inspectron::PolicyDecision stop_decision{
        .action = inspectron::RecommendedAction::Stop,
        .reason = inspectron::DecisionReason::InvalidAssessment,
        .model_action_overridden = false,
    };

    auto output = to_message(
        stop_decision, DecisionMessage::STATUS_STALE, evidence_id);
    output.source_observed_at = source_observed_at;
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
  builtin_interfaces::msg::Time last_observed_at_{};
  std::int64_t recovery_barrier_ns_{0};
  std::set<std::int64_t> rejected_future_observations_;
  std::chrono::steady_clock::time_point
      rejected_future_quarantine_until_{};

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
