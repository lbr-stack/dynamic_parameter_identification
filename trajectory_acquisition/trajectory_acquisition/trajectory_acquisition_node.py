from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from lbr_fri_msgs.msg import LBRState
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.parameter import Parameter


class TrajectoryAcquisitionNode(Node):
    def __init__(self, node_name: str = "trajectory_acquisition_node") -> None:
        super().__init__(node_name)
        self.lbr_state_ = None
        self.lbr_state_sub_ = self.create_subscription(
            LBRState,
            "/lbr_state",
            self.lbr_state_sub_sb_,
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

        self.declare_parameter("target_joint_positions", Parameter.Type.DOUBLE_ARRAY)
        self.target_joint_positions_ = self.get_parameter("target_joint_positions")

        # print(self.target_joint_positions_)

    def execute(self) -> None:

        rclpy.spin_until_future_complete(self)

    def lbr_state_sub_sb_(self, lbr_state: LBRState) -> None:
        self.lbr_state_ = lbr_state


def main(args: List = None) -> None:
    rclpy.init(args=args)
    trajectory_acquisition_node = TrajectoryAcquisitionNode()
    trajectory_acquisition_node.execute()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
