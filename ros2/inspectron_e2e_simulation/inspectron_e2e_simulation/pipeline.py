from __future__ import annotations

from launch_ros.actions import Node

PRODUCTION_TOPICS = (
    "/inspectron/camera/compressed",
    "/inspectron/scene_assessment",
    "/inspectron/evidence_capture",
    "/inspectron/policy_decision",
    "/inspectron/desired_cmd_vel",
    "/cmd_vel",
    "/inspectron/emergency_stop",
    "/inspectron/mission/state",
    "/inspectron/mission/waypoint_goal",
    "/inspectron/mission/cancel_motion",
    "/inspectron/mission/perception_request",
    "/inspectron/mission/closer_view_request",
    "/inspectron/mission/reroute_request",
    "/inspectron/mission/waypoint_reached",
    "/inspectron/mission/reroute_target",
    "/inspectron/report/status",
)

PRODUCTION_SERVICES = (
    "/inspectron/mission/start",
    "/inspectron/mission/pause",
    "/inspectron/mission/resume",
    "/inspectron/mission/abort",
    "/inspectron/mission/reset",
    "/inspectron/report/finalize",
)


def remapped_name(name: str, prefix: str) -> str:
    if not prefix:
        return name

    return f"{prefix}{name}"


def pipeline_remappings(prefix: str) -> list[tuple[str, str]]:
    if not prefix:
        return []

    return [
        (name, remapped_name(name, prefix)) for name in (*PRODUCTION_TOPICS, *PRODUCTION_SERVICES)
    ]


def _node_name(base: str, suffix: str) -> str:
    if not suffix:
        return base

    return f"{base}_{suffix}"


def create_pipeline_nodes(
    *,
    waypoints: list[str],
    fixture_response_json: str,
    report_directory: str,
    scene_id: str,
    desired_velocity_mode: str,
    auto_start: bool,
    abort_on_safety_stop: bool,
    prefix: str = "",
    node_name_suffix: str = "",
    finalize_delay_ms: int = 250,
) -> list[Node]:
    remappings = pipeline_remappings(prefix)

    perception_bridge = Node(
        package="inspectron_perception_bridge",
        executable="perception_bridge_node",
        name=_node_name("perception_bridge", node_name_suffix),
        output="screen",
        parameters=[
            {
                "provider": "fixture",
                "model": "fixture-model",
                "fixture_response_json": fixture_response_json,
                "default_waypoint": "unknown_waypoint",
                "scene_id": scene_id,
                "timeout_seconds": 5.0,
                "max_image_bytes": 65536,
            }
        ],
        remappings=remappings,
    )

    safety_supervisor = Node(
        package="inspectron_safety_supervisor",
        executable="safety_supervisor_node",
        name=_node_name("safety_supervisor", node_name_suffix),
        output="screen",
        parameters=[
            {
                "confidence_threshold": 0.70,
                "view_quality_threshold": 0.60,
                "assessment_timeout_ms": 3000,
            }
        ],
        remappings=remappings,
    )

    mission_orchestrator = Node(
        package="inspectron_mission_orchestrator",
        executable="mission_orchestrator_node",
        name=_node_name("mission_orchestrator", node_name_suffix),
        output="screen",
        parameters=[
            {
                "waypoints": waypoints,
                "policy_timeout_ms": 1500,
                "reroute_timeout_ms": 5000,
                "watchdog_rate_hz": 20.0,
            }
        ],
        remappings=remappings,
    )

    motion_controller = Node(
        package="inspectron_motion_controller",
        executable="motion_controller_node",
        name=_node_name("motion_controller", node_name_suffix),
        output="screen",
        parameters=[
            {
                "policy_timeout_ms": 1500,
                "command_timeout_ms": 250,
                "slow_scale": 0.35,
                "max_linear_speed": 0.40,
                "max_angular_speed": 0.80,
                "control_rate_hz": 20.0,
            }
        ],
        remappings=remappings,
    )

    evidence_reporter = Node(
        package="inspectron_evidence_reporter",
        executable="evidence_reporter_node",
        name=_node_name("evidence_reporter", node_name_suffix),
        output="screen",
        parameters=[
            {
                "output_directory": report_directory,
                "max_image_bytes": 65536,
                "auto_finalize": True,
                "write_partial_report": True,
                "finalize_delay_ms": finalize_delay_ms,
            }
        ],
        remappings=remappings,
    )

    scenario_simulator = Node(
        package="inspectron_e2e_simulation",
        executable="scenario_simulator_node",
        name=_node_name("scenario_simulator", node_name_suffix),
        output="screen",
        parameters=[
            {
                "waypoints": waypoints,
                "desired_linear_x": 0.30,
                "desired_angular_z": 0.0,
                "min_authorized_motion_samples": 5,
                "frame_interval_ms": 125,
                "desired_velocity_mode": desired_velocity_mode,
                "auto_start": auto_start,
                "abort_on_safety_stop": abort_on_safety_stop,
                "tick_rate_hz": 20.0,
            }
        ],
        remappings=remappings,
    )

    return [
        perception_bridge,
        safety_supervisor,
        mission_orchestrator,
        motion_controller,
        evidence_reporter,
        scenario_simulator,
    ]
