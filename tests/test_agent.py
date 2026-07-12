import unittest

from inspectron.agent import InspectionAgent
from inspectron.domain import ActionKind, DefectType, Observation


class InspectionAgentTests(unittest.TestCase):
    def test_requests_another_view_for_weak_evidence(self) -> None:
        agent = InspectionAgent(["bay_a"])

        observation = Observation(
            waypoint="bay_a",
            asset_id="column_a",
            evidence_id="image_a_0",
            predicted_defect=DefectType.CRACK,
            confidence=0.55,
            view_quality=0.40,
        )

        agent.observe(observation)
        action = agent.choose_action(observation)

        self.assertEqual(action.kind, ActionKind.INSPECT)
        self.assertEqual(agent.findings, {})

    def test_records_supported_finding(self) -> None:
        agent = InspectionAgent(["bay_a"])

        observation = Observation(
            waypoint="bay_a",
            asset_id="column_a",
            evidence_id="image_a_0",
            predicted_defect=DefectType.CRACK,
            confidence=0.93,
            view_quality=0.89,
        )

        agent.observe(observation)

        finding = next(iter(agent.findings.values()))

        self.assertEqual(finding.defect_type, DefectType.CRACK)
        self.assertEqual(finding.evidence_ids, ("image_a_0",))


if __name__ == "__main__":
    unittest.main()