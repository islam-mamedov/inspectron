#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "inspectron/safety_policy.hpp"
#include "inspectron_safety_supervisor/message_conversion.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "inspectron_safety_supervisor/msg/scene_assessment.hpp"

namespace inspectron_safety_supervisor {
namespace {

using SceneMessage = msg::SceneAssessment;
using DecisionMessage = msg::PolicyDecision;

TEST(MessageConversionTest, MessageConstantsMatchCoreEnums) {
  EXPECT_EQ(
      SceneMessage::TRAVERSABILITY_CLEAR,
      static_cast<std::uint8_t>(inspectron::Traversability::Clear));
  EXPECT_EQ(
      SceneMessage::HAZARD_OPEN_EDGE,
      static_cast<std::uint8_t>(inspectron::HazardType::OpenEdge));
  EXPECT_EQ(
      SceneMessage::ACTION_INSPECT_CLOSER,
      static_cast<std::uint8_t>(
          inspectron::RecommendedAction::InspectCloser));
  EXPECT_EQ(
      DecisionMessage::REASON_CRITICAL_HAZARD,
      static_cast<std::uint8_t>(inspectron::DecisionReason::CriticalHazard));
}

TEST(MessageConversionTest, ConvertsStructuredAssessment) {
  SceneMessage input;
  input.traversability = SceneMessage::TRAVERSABILITY_RESTRICTED;
  input.hazards = {
      SceneMessage::HAZARD_DEBRIS,
      SceneMessage::HAZARD_LIQUID_SPILL,
  };
  input.recommended_action = SceneMessage::ACTION_SLOW_DOWN;
  input.confidence = 0.91;
  input.view_quality = 0.82;

  const auto converted = from_message(input);

  EXPECT_EQ(converted.traversability, inspectron::Traversability::Restricted);
  ASSERT_EQ(converted.hazards.size(), 2U);
  EXPECT_EQ(converted.hazards[0], inspectron::HazardType::Debris);
  EXPECT_EQ(converted.hazards[1], inspectron::HazardType::LiquidSpill);
  EXPECT_EQ(
      converted.recommended_action,
      inspectron::RecommendedAction::SlowDown);
  EXPECT_DOUBLE_EQ(converted.confidence, 0.91);
  EXPECT_DOUBLE_EQ(converted.view_quality, 0.82);
}

TEST(MessageConversionTest, UnknownEnumReachesFailClosedCoreValidation) {
  SceneMessage input;
  input.traversability = std::numeric_limits<std::uint8_t>::max();
  input.recommended_action = SceneMessage::ACTION_PROCEED;
  input.confidence = 0.95;
  input.view_quality = 0.95;

  const auto converted = from_message(input);
  const auto decision = inspectron::resolve_policy(converted);

  EXPECT_FALSE(inspectron::is_valid_assessment(converted));
  EXPECT_EQ(decision.action, inspectron::RecommendedAction::Stop);
  EXPECT_EQ(decision.reason, inspectron::DecisionReason::InvalidAssessment);
}

TEST(MessageConversionTest, ConvertsAuditablePolicyDecision) {
  const inspectron::PolicyDecision decision{
      .action = inspectron::RecommendedAction::Stop,
      .reason = inspectron::DecisionReason::CriticalHazard,
      .model_action_overridden = true,
  };

  const auto output = to_message(
      decision, DecisionMessage::STATUS_VALID, "frame_0042");

  EXPECT_EQ(output.action, DecisionMessage::ACTION_STOP);
  EXPECT_EQ(output.reason, DecisionMessage::REASON_CRITICAL_HAZARD);
  EXPECT_EQ(output.status, DecisionMessage::STATUS_VALID);
  EXPECT_TRUE(output.model_action_overridden);
  EXPECT_EQ(output.evidence_id, std::string{"frame_0042"});
}

}  // namespace
}  // namespace inspectron_safety_supervisor
