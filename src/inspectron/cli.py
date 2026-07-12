from __future__ import annotations

import json

from inspectron.evaluation import run_episode, score_episode
from inspectron.simulation import baseline_scenario


def main() -> None:
    scenario = baseline_scenario()
    result = run_episode(scenario)
    metrics = score_episode(result, scenario.ground_truth)

    output = {
        "scenario": scenario.name,
        "metrics": metrics.to_dict(),
        "findings": [
            {
                "asset_id": finding.asset_id,
                "defect_type": finding.defect_type,
                "confidence": finding.confidence,
                "evidence_ids": finding.evidence_ids,
            }
            for finding in result.findings
        ],
        "action_trace": [
            {
                "action": action.kind,
                "target": action.target,
                "reason": action.reason,
            }
            for action in result.action_trace
        ],
    }

    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()