from __future__ import annotations

from inspectron_e2e_simulation.pipeline import create_pipeline_nodes
from inspectron_e2e_simulation.scenarios import scenario_definition
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def _create_scenario_actions(context):
    scenario_name = LaunchConfiguration("scenario").perform(context)
    report_directory = LaunchConfiguration("report_directory").perform(context)

    scenario = scenario_definition(scenario_name)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=report_directory,
        scene_id=scenario.scene_id,
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=True,
        abort_on_safety_stop=scenario.abort_on_safety_stop,
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=[Shutdown(reason="an Inspectron simulation process exited")],
        )
    )

    return [*nodes, shutdown_on_exit]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="safe_mission",
                description="Scenario to run: safe_mission or hazard_stop",
                choices=["safe_mission", "hazard_stop"],
            ),
            DeclareLaunchArgument(
                "report_directory",
                default_value="/tmp/inspectron_e2e_reports",
                description="Directory that receives mission report artifacts",
            ),
            OpaqueFunction(function=_create_scenario_actions),
        ]
    )
