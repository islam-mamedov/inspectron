# Enforced actions. Numeric values match inspectron::RecommendedAction.
uint8 ACTION_PROCEED=0
uint8 ACTION_SLOW_DOWN=1
uint8 ACTION_STOP=2
uint8 ACTION_REROUTE=3
uint8 ACTION_INSPECT_CLOSER=4

# Stable decision reasons. Numeric values match inspectron::DecisionReason.
uint8 REASON_INVALID_ASSESSMENT=0
uint8 REASON_CRITICAL_HAZARD=1
uint8 REASON_BLOCKED_PATH=2
uint8 REASON_WEAK_EVIDENCE=3
uint8 REASON_RESTRICTED_PATH=4
uint8 REASON_CLEAR_PATH=5

# Input-health status is separate from the policy reason.
uint8 STATUS_VALID=0
uint8 STATUS_INVALID=1
uint8 STATUS_STALE=2

uint8 action
uint8 reason
uint8 status
bool model_action_overridden
builtin_interfaces/Time source_observed_at
builtin_interfaces/Time decided_at
string evidence_id
