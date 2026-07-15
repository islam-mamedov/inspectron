# Traversability values. Numeric values match inspectron::Traversability.
uint8 TRAVERSABILITY_CLEAR=0
uint8 TRAVERSABILITY_RESTRICTED=1
uint8 TRAVERSABILITY_BLOCKED=2
uint8 TRAVERSABILITY_UNKNOWN=3

# Hazard values. Numeric values match inspectron::HazardType.
uint8 HAZARD_HUMAN_IN_PATH=0
uint8 HAZARD_DEBRIS=1
uint8 HAZARD_LIQUID_SPILL=2
uint8 HAZARD_OPEN_EDGE=3
uint8 HAZARD_FIRE_OR_SMOKE=4
uint8 HAZARD_UNSTABLE_LOAD=5

# Model-recommended actions. Numeric values match inspectron::RecommendedAction.
uint8 ACTION_PROCEED=0
uint8 ACTION_SLOW_DOWN=1
uint8 ACTION_STOP=2
uint8 ACTION_REROUTE=3
uint8 ACTION_INSPECT_CLOSER=4

uint8 traversability
uint8[] hazards
uint8 recommended_action
float64 confidence
float64 view_quality
builtin_interfaces/Time observed_at
string evidence_id
