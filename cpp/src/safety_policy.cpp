#include "inspectron/safety_policy.hpp"

#include <algorithm>
#include <cmath>

namespace inspectron {
namespace {

[[nodiscard]] bool is_probability(double value) noexcept {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

[[nodiscard]] bool is_known(Traversability value) noexcept {
  switch (value) {
    case Traversability::Clear:
    case Traversability::Restricted:
    case Traversability::Blocked:
    case Traversability::Unknown:
      return true;
  }
  return false;
}

[[nodiscard]] bool is_known(HazardType value) noexcept {
  switch (value) {
    case HazardType::HumanInPath:
    case HazardType::Debris:
    case HazardType::LiquidSpill:
    case HazardType::OpenEdge:
    case HazardType::FireOrSmoke:
    case HazardType::UnstableLoad:
      return true;
  }
  return false;
}

[[nodiscard]] bool is_known(RecommendedAction value) noexcept {
  switch (value) {
    case RecommendedAction::Proceed:
    case RecommendedAction::SlowDown:
    case RecommendedAction::Stop:
    case RecommendedAction::Reroute:
    case RecommendedAction::InspectCloser:
      return true;
  }
  return false;
}

[[nodiscard]] bool has_critical_hazard(
    const std::vector<HazardType>& hazards) noexcept {
  return std::ranges::any_of(hazards, is_critical_hazard);
}

[[nodiscard]] PolicyDecision decision(
    const SceneAssessment& assessment,
    RecommendedAction action,
    DecisionReason reason) noexcept {
  return PolicyDecision{
      .action = action,
      .reason = reason,
      .model_action_overridden = assessment.recommended_action != action,
  };
}

}  // namespace

bool is_critical_hazard(HazardType hazard) noexcept {
  switch (hazard) {
    case HazardType::HumanInPath:
    case HazardType::OpenEdge:
    case HazardType::FireOrSmoke:
    case HazardType::UnstableLoad:
      return true;
    case HazardType::Debris:
    case HazardType::LiquidSpill:
      return false;
  }
  return true;
}

bool is_valid_assessment(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds) noexcept {
  if (!is_probability(assessment.confidence) ||
      !is_probability(assessment.view_quality) ||
      !is_probability(thresholds.confidence) ||
      !is_probability(thresholds.view_quality) ||
      !is_known(assessment.traversability) ||
      !is_known(assessment.recommended_action)) {
    return false;
  }

  return std::ranges::all_of(assessment.hazards, [](HazardType hazard) {
    return is_known(hazard);
  });
}

PolicyDecision resolve_policy(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds) noexcept {
  if (!is_valid_assessment(assessment, thresholds)) {
    return decision(
        assessment,
        RecommendedAction::Stop,
        DecisionReason::InvalidAssessment);
  }

  if (has_critical_hazard(assessment.hazards)) {
    return decision(
        assessment,
        RecommendedAction::Stop,
        DecisionReason::CriticalHazard);
  }

  if (assessment.traversability == Traversability::Blocked) {
    return decision(
        assessment,
        RecommendedAction::Reroute,
        DecisionReason::BlockedPath);
  }

  const bool evidence_is_weak =
      assessment.confidence < thresholds.confidence ||
      assessment.view_quality < thresholds.view_quality ||
      assessment.traversability == Traversability::Unknown;

  if (evidence_is_weak) {
    return decision(
        assessment,
        RecommendedAction::InspectCloser,
        DecisionReason::WeakEvidence);
  }

  if (assessment.traversability == Traversability::Restricted ||
      !assessment.hazards.empty()) {
    return decision(
        assessment,
        RecommendedAction::SlowDown,
        DecisionReason::RestrictedPath);
  }

  return decision(
      assessment,
      RecommendedAction::Proceed,
      DecisionReason::ClearPath);
}

RecommendedAction resolve_safe_action(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds) noexcept {
  return resolve_policy(assessment, thresholds).action;
}

std::vector<std::string_view> find_consistency_violations(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds) {
  std::vector<std::string_view> violations;

  if (!is_valid_assessment(assessment, thresholds)) {
    violations.emplace_back("invalid_scene_assessment");
    return violations;
  }

  if (assessment.traversability == Traversability::Clear &&
      !assessment.hazards.empty()) {
    violations.emplace_back("clear_traversability_with_reported_hazards");
  }

  if (assessment.recommended_action !=
      resolve_safe_action(assessment, thresholds)) {
    violations.emplace_back("model_action_disagrees_with_safety_policy");
  }

  return violations;
}

std::string_view to_string(Traversability value) noexcept {
  switch (value) {
    case Traversability::Clear:
      return "clear";
    case Traversability::Restricted:
      return "restricted";
    case Traversability::Blocked:
      return "blocked";
    case Traversability::Unknown:
      return "unknown";
  }
  return "invalid";
}

std::string_view to_string(HazardType value) noexcept {
  switch (value) {
    case HazardType::HumanInPath:
      return "human_in_path";
    case HazardType::Debris:
      return "debris";
    case HazardType::LiquidSpill:
      return "liquid_spill";
    case HazardType::OpenEdge:
      return "open_edge";
    case HazardType::FireOrSmoke:
      return "fire_or_smoke";
    case HazardType::UnstableLoad:
      return "unstable_load";
  }
  return "invalid";
}

std::string_view to_string(RecommendedAction value) noexcept {
  switch (value) {
    case RecommendedAction::Proceed:
      return "proceed";
    case RecommendedAction::SlowDown:
      return "slow_down";
    case RecommendedAction::Stop:
      return "stop";
    case RecommendedAction::Reroute:
      return "reroute";
    case RecommendedAction::InspectCloser:
      return "inspect_closer";
  }
  return "invalid";
}

std::string_view to_string(DecisionReason value) noexcept {
  switch (value) {
    case DecisionReason::InvalidAssessment:
      return "invalid_assessment";
    case DecisionReason::CriticalHazard:
      return "critical_hazard";
    case DecisionReason::BlockedPath:
      return "blocked_path";
    case DecisionReason::WeakEvidence:
      return "weak_evidence";
    case DecisionReason::RestrictedPath:
      return "restricted_path";
    case DecisionReason::ClearPath:
      return "clear_path";
  }
  return "invalid";
}

}  // namespace inspectron
