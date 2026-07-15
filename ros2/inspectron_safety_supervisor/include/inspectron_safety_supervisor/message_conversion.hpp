#pragma once

#include <cstdint>
#include <string_view>

#include "inspectron/safety_policy.hpp"
#include "inspectron_safety_supervisor/msg/policy_decision.hpp"
#include "inspectron_safety_supervisor/msg/scene_assessment.hpp"

namespace inspectron_safety_supervisor {

namespace message = inspectron_safety_supervisor::msg;

[[nodiscard]] inline inspectron::SceneAssessment from_message(
    const message::SceneAssessment& input) {
  inspectron::SceneAssessment assessment{
      .traversability =
          static_cast<inspectron::Traversability>(input.traversability),
      .hazards = {},
      .recommended_action =
          static_cast<inspectron::RecommendedAction>(input.recommended_action),
      .confidence = input.confidence,
      .view_quality = input.view_quality,
  };

  assessment.hazards.reserve(input.hazards.size());
  for (const auto hazard : input.hazards) {
    assessment.hazards.push_back(
        static_cast<inspectron::HazardType>(hazard));
  }

  return assessment;
}

[[nodiscard]] inline message::PolicyDecision to_message(
    const inspectron::PolicyDecision& decision,
    std::uint8_t status,
    std::string_view evidence_id) {
  message::PolicyDecision output;
  output.action = static_cast<std::uint8_t>(decision.action);
  output.reason = static_cast<std::uint8_t>(decision.reason);
  output.status = status;
  output.model_action_overridden = decision.model_action_overridden;
  output.evidence_id = evidence_id;
  return output;
}

}  // namespace inspectron_safety_supervisor
