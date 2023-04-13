import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:

    trajectory_execution_node = Node(
        package="trajectory_acquisition",
        executable="trajectory_execution_node",
        output="screen",
    )

    return LaunchDescription([trajectory_execution_node])
