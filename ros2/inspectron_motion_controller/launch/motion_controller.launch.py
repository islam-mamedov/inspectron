from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("inspectron_motion_controller"))
    parameters = package_share / "config" / "motion_controller.yaml"

    return LaunchDescription(
        [
            Node(
                package="inspectron_motion_controller",
                executable="motion_controller_node",
                name="motion_controller",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
