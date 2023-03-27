import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:

    trajectory_acquisition_config = os.path.join(
        get_package_share_directory("trajectory_acquisition"),
        "config",
        "trajectory_acquisition.yml",
    )
    trajectory_acquisition_node = Node(
        package="trajectory_acquisition",
        executable="trajectory_acquisition_node",
        parameters=[trajectory_acquisition_config],
    )

    return LaunchDescription([trajectory_acquisition_node])
