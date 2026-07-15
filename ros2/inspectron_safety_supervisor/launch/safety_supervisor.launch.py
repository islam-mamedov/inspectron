from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("inspectron_safety_supervisor"))
    configuration = package_share / "config" / "safety_supervisor.yaml"

    return LaunchDescription(
        [
            Node(
                package="inspectron_safety_supervisor",
                executable="safety_supervisor_node",
                name="safety_supervisor",
                output="screen",
                parameters=[str(configuration)],
            )
        ]
    )
