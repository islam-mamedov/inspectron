#include "inspectron/safety_policy.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string_view>
#include <utility>
#include <vector>

namespace inspectron {
namespace {

[[nodiscard]] SceneAssessment assessment(
    Traversability traversability,
    std::vector<HazardType> hazards = {},
    RecommendedAction action = RecommendedAction::Proceed,
    double confidence = 0.90,
    double view_quality = 0.90) {
  return SceneAssessment{
      .traversability = traversability,
      .hazards = std::move(hazards),
      .recommended_action = action,
      .confidence = confidence,
      .view_quality = view_quality,
  };
}

TEST(SafetyPolicyTest, EveryCriticalHazardForcesStop) {
  for (const auto hazard : {
           HazardType::HumanInPath,
           HazardType::OpenEdge,
           HazardType::FireOrSmoke,
           HazardType::UnstableLoad,
       }) {
    const auto scene = assessment(Traversability::Clear, {hazard});
    const auto result = resolve_policy(scene);

    EXPECT_EQ(result.action, RecommendedAction::Stop);
    EXPECT_EQ(result.reason, DecisionReason::CriticalHazard);
    EXPECT_TRUE(result.model_action_overridden);
  }
}

TEST(SafetyPolicyTest, CriticalHazardTakesPriorityOverBlockedPath) {
  const auto scene = assessment(
      Traversability::Blocked,
      {HazardType::FireOrSmoke},
      RecommendedAction::Reroute);

  const auto result = resolve_policy(scene);

  EXPECT_EQ(result.action, RecommendedAction::Stop);
  EXPECT_EQ(result.reason, DecisionReason::CriticalHazard);
}

TEST(SafetyPolicyTest, BlockedPathForcesReroute) {
  const auto scene = assessment(Traversability::Blocked);

  const auto result = resolve_policy(scene);

  EXPECT_EQ(result.action, RecommendedAction::Reroute);
  EXPECT_EQ(result.reason, DecisionReason::BlockedPath);
}

TEST(SafetyPolicyTest, UnknownTraversabilityRequestsCloserInspection) {
  const auto scene = assessment(
      Traversability::Unknown,
      {},
      RecommendedAction::Proceed,
      0.95,
      0.95);

  const auto result = resolve_policy(scene);

  EXPECT_EQ(result.action, RecommendedAction::InspectCloser);
  EXPECT_EQ(result.reason, DecisionReason::WeakEvidence);
}

TEST(SafetyPolicyTest, LowConfidenceRequestsCloserInspection) {
  const auto scene = assessment(
      Traversability::Clear,
      {},
      RecommendedAction::Proceed,
      0.69,
      0.95);

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::InspectCloser);
}

TEST(SafetyPolicyTest, LowViewQualityRequestsCloserInspection) {
  const auto scene = assessment(
      Traversability::Clear,
      {},
      RecommendedAction::Proceed,
      0.95,
      0.59);

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::InspectCloser);
}

TEST(SafetyPolicyTest, ThresholdValuesAreAccepted) {
  const auto scene = assessment(
      Traversability::Clear,
      {},
      RecommendedAction::Proceed,
      0.70,
      0.60);

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::Proceed);
}

TEST(SafetyPolicyTest, RestrictedPathSlowsDown) {
  const auto scene = assessment(
      Traversability::Restricted,
      {},
      RecommendedAction::SlowDown);

  const auto result = resolve_policy(scene);

  EXPECT_EQ(result.action, RecommendedAction::SlowDown);
  EXPECT_EQ(result.reason, DecisionReason::RestrictedPath);
  EXPECT_FALSE(result.model_action_overridden);
}

TEST(SafetyPolicyTest, NonCriticalHazardSlowsDown) {
  for (const auto hazard : {
           HazardType::Debris,
           HazardType::LiquidSpill,
       }) {
    const auto scene = assessment(
        Traversability::Clear,
        {hazard},
        RecommendedAction::SlowDown);

    EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::SlowDown);
  }
}

TEST(SafetyPolicyTest, ClearSafeSceneProceeds) {
  const auto scene = assessment(Traversability::Clear);

  const auto result = resolve_policy(scene);

  EXPECT_EQ(result.action, RecommendedAction::Proceed);
  EXPECT_EQ(result.reason, DecisionReason::ClearPath);
  EXPECT_FALSE(result.model_action_overridden);
}

TEST(SafetyPolicyTest, InvalidProbabilityFailsClosed) {
  for (const auto value : {
           -0.01,
           1.01,
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::infinity(),
       }) {
    auto scene = assessment(Traversability::Clear);
    scene.confidence = value;

    const auto result = resolve_policy(scene);

    EXPECT_EQ(result.action, RecommendedAction::Stop);
    EXPECT_EQ(result.reason, DecisionReason::InvalidAssessment);
  }
}

TEST(SafetyPolicyTest, InvalidViewQualityFailsClosed) {
  auto scene = assessment(Traversability::Clear);
  scene.view_quality = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::Stop);
}

TEST(SafetyPolicyTest, InvalidThresholdsFailClosed) {
  const auto scene = assessment(Traversability::Clear);

  const auto result = resolve_policy(
      scene,
      PolicyThresholds{.confidence = 1.5, .view_quality = 0.60});

  EXPECT_EQ(result.action, RecommendedAction::Stop);
  EXPECT_EQ(result.reason, DecisionReason::InvalidAssessment);
}

TEST(SafetyPolicyTest, InvalidEnumFailsClosed) {
  auto scene = assessment(Traversability::Clear);
  scene.traversability = static_cast<Traversability>(255);

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::Stop);

  scene = assessment(Traversability::Clear);
  scene.hazards = {static_cast<HazardType>(255)};

  EXPECT_EQ(resolve_safe_action(scene), RecommendedAction::Stop);
}

TEST(SafetyPolicyTest, ClearSceneWithHazardIsContradictory) {
  const auto scene = assessment(
      Traversability::Clear,
      {HazardType::Debris},
      RecommendedAction::SlowDown);

  const auto violations = find_consistency_violations(scene);

  ASSERT_EQ(violations.size(), 1U);
  EXPECT_EQ(
      violations.front(),
      std::string_view{"clear_traversability_with_reported_hazards"});
}

TEST(SafetyPolicyTest, ModelPolicyDisagreementIsReported) {
  const auto scene = assessment(
      Traversability::Blocked,
      {},
      RecommendedAction::Proceed);

  const auto violations = find_consistency_violations(scene);

  ASSERT_EQ(violations.size(), 1U);
  EXPECT_EQ(
      violations.front(),
      std::string_view{"model_action_disagrees_with_safety_policy"});
}

TEST(SafetyPolicyTest, InvalidAssessmentViolationIsReported) {
  auto scene = assessment(Traversability::Clear);
  scene.confidence = std::numeric_limits<double>::quiet_NaN();

  const auto violations = find_consistency_violations(scene);

  ASSERT_EQ(violations.size(), 1U);
  EXPECT_EQ(
      violations.front(),
      std::string_view{"invalid_scene_assessment"});
}

TEST(SafetyPolicyTest, StringValuesMatchPythonSchema) {
  EXPECT_EQ(to_string(Traversability::Restricted), "restricted");
  EXPECT_EQ(to_string(HazardType::HumanInPath), "human_in_path");
  EXPECT_EQ(to_string(HazardType::FireOrSmoke), "fire_or_smoke");
  EXPECT_EQ(to_string(RecommendedAction::InspectCloser), "inspect_closer");
  EXPECT_EQ(to_string(DecisionReason::InvalidAssessment), "invalid_assessment");
}

TEST(SafetyPolicyTest, UnknownStringValuesAreInvalid) {
  EXPECT_EQ(to_string(static_cast<Traversability>(255)), "invalid");
  EXPECT_EQ(to_string(static_cast<HazardType>(255)), "invalid");
  EXPECT_EQ(to_string(static_cast<RecommendedAction>(255)), "invalid");
  EXPECT_EQ(to_string(static_cast<DecisionReason>(255)), "invalid");
}

}  // namespace
}  // namespace inspectron
