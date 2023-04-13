from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryExecutionNode(Node):
    def __init__(self, node_name: str = "trajectory_execution_node") -> None:
        super().__init__(node_name)

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

    def go_to(
        self,
        joint_names: List[str] = [
            "lbr_joint_0",
            "lbr_joint_1",
            "lbr_joint_2",
            "lbr_joint_3",
            "lbr_joint_4",
            "lbr_joint_5",
            "lbr_joint_6",
        ],
        target_joint_position: List[float] = [0.0] * 7,
        time_from_start: Duration = Duration(
            sec=1,
            nanosec=0,
        ),
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance.sec = 1
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=target_joint_position, time_from_start=time_from_start
            )
        )
        goal_handle_future = self.follow_joint_trajetory_ac_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle_future)

        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return
        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Got result: {result.error_code}")


import csv
import os

from ament_index_python import get_package_share_directory


def main(args: List = None) -> None:
    rclpy.init(args=args)
    trajectory_acquisition_node = TrajectoryExecutionNode()

    # read target positions from csv
    with open(
        os.path.join(
            get_package_share_directory("trajectory_acquisition"),
            "config",
            "target_joint_positions.csv",
        ),
        "r",
    ) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            joint_names = [x.strip() for x in list(row.keys())]
            target_positions = [float(x) for x in row.values()]

            # execute
            trajectory_acquisition_node.go_to(
                joint_names=joint_names,
                target_joint_position=target_positions,
                time_from_start=Duration(sec=4, nanosec=0),
            )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
