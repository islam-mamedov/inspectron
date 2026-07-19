from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("inspectron_evidence_reporter"))
    parameters = package_share / "config" / "evidence_reporter.yaml"

    return LaunchDescription(
        [
            Node(
                package="inspectron_evidence_reporter",
                executable="evidence_reporter_node",
                name="evidence_reporter",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
