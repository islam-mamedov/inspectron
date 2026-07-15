#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

namespace inspectron {

enum class Traversability : std::uint8_t {
  Clear,
  Restricted,
  Blocked,
  Unknown,
};

enum class HazardType : std::uint8_t {
  HumanInPath,
  Debris,
  LiquidSpill,
  OpenEdge,
  FireOrSmoke,
  UnstableLoad,
};

enum class RecommendedAction : std::uint8_t {
  Proceed,
  SlowDown,
  Stop,
  Reroute,
  InspectCloser,
};

enum class DecisionReason : std::uint8_t {
  InvalidAssessment,
  CriticalHazard,
  BlockedPath,
  WeakEvidence,
  RestrictedPath,
  ClearPath,
};

struct SceneAssessment {
  Traversability traversability{Traversability::Unknown};
  std::vector<HazardType> hazards{};
  RecommendedAction recommended_action{RecommendedAction::InspectCloser};
  double confidence{0.0};
  double view_quality{0.0};
};

struct PolicyThresholds {
  double confidence{0.70};
  double view_quality{0.60};
};

struct PolicyDecision {
  RecommendedAction action{RecommendedAction::Stop};
  DecisionReason reason{DecisionReason::InvalidAssessment};
  bool model_action_overridden{false};
};

[[nodiscard]] bool is_critical_hazard(HazardType hazard) noexcept;

[[nodiscard]] bool is_valid_assessment(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds = {}) noexcept;

[[nodiscard]] PolicyDecision resolve_policy(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds = {}) noexcept;

[[nodiscard]] RecommendedAction resolve_safe_action(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds = {}) noexcept;

[[nodiscard]] std::vector<std::string_view> find_consistency_violations(
    const SceneAssessment& assessment,
    PolicyThresholds thresholds = {});

[[nodiscard]] std::string_view to_string(Traversability value) noexcept;
[[nodiscard]] std::string_view to_string(HazardType value) noexcept;
[[nodiscard]] std::string_view to_string(RecommendedAction value) noexcept;
[[nodiscard]] std::string_view to_string(DecisionReason value) noexcept;

}  // namespace inspectron
