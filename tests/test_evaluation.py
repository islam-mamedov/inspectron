import unittest

from inspectron.domain import ActionKind, DefectType
from inspectron.evaluation import run_episode, score_episode
from inspectron.simulation import baseline_scenario


class EvaluationTests(unittest.TestCase):
    def test_baseline_recovers_planted_defects(self) -> None:
        scenario = baseline_scenario()

        result = run_episode(scenario)
        metrics = score_episode(result, scenario.ground_truth)

        self.assertTrue(metrics.completed)
        self.assertEqual(metrics.coverage, 1.0)
        self.assertEqual(metrics.precision, 1.0)
        self.assertEqual(metrics.recall, 1.0)
        self.assertEqual(metrics.safety_violations, 0)

        actions = [action.kind for action in result.action_trace]
        self.assertIn(ActionKind.INSPECT, actions)

        findings = {
            (finding.asset_id, finding.defect_type)
            for finding in result.findings
        }

        self.assertEqual(
            findings,
            {
                ("column_a", DefectType.CRACK),
                ("column_c", DefectType.CORROSION),
            },
        )


if __name__ == "__main__":
    unittest.main()