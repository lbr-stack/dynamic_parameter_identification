from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    joint_state_recording_node = Node(
        package="trajectory_acquisition",
        executable="joint_state_recording_node",
        output="screen",
    )

    trajectory_execution_node = Node(
        package="trajectory_acquisition",
        executable="trajectory_execution_node",
        output="screen",
    )

    return LaunchDescription([trajectory_execution_node, joint_state_recording_node])
