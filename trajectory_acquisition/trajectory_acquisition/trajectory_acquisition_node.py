from typing import List

import time
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryAcquisitionNode(Node):
    def __init__(self, node_name: str = "trajectory_acquisition_node") -> None:
        super().__init__(node_name)
        self.joint_state_ = None
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_sub_sb_,
            qos_profile_sensor_data,
        )

        self.follow_joint_trajetory_ac_name_ = (
            "/position_trajectory_controller/follow_joint_trajectory"
        )
        self.follow_joint_trajetory_ac_ = ActionClient(
            self,
            FollowJointTrajectory,
            self.follow_joint_trajetory_ac_name_,
        )
        self.get_logger().info(
            f"Waiting for {self.follow_joint_trajetory_ac_name_} server..."
        )
        self.follow_joint_trajetory_ac_.wait_for_server()
        self.get_logger().info("Done.")

        # self.declare_parameter("target_joint_positions", Parameter.Type.DOUBLE_ARRAY)
        # self.target_joint_positions_ = self.get_parameter("target_joint_positions")

        # print(self.target_joint_positions_)

    def execute(self) -> None:
        while not self.joint_state_:
            rclpy.spin_once(self)
            time.sleep(0.1)

        dof = len(self.joint_state_.name)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = self.joint_state_.name
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=[0.0] * dof,
                velocities=[0.0] * dof,
                accelerations=[0.0] * dof,
                time_from_start=Duration(
                    sec=10,
                    nanosec=0,
                ),
            )
        )
        future = self.follow_joint_trajetory_ac_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

    def joint_state_sub_sb_(self, joint_state: JointState) -> None:
        self.joint_state_ = joint_state


def main(args: List = None) -> None:
    rclpy.init(args=args)
    trajectory_acquisition_node = TrajectoryAcquisitionNode()
    trajectory_acquisition_node.execute()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
