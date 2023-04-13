from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from trajectory_acquisition_msgs.srv import Save


class TrajectoryExecutionNode(Node):
    def __init__(self, node_name: str = "trajectory_execution_node") -> None:
        super().__init__(node_name)

        # trajectory execution action client
        self.follow_joint_trajetory_ac_name_ = (
            "/position_trajectory_controller/follow_joint_trajectory"
        )
        self.follow_joint_trajetory_ac_ = ActionClient(
            self,
            FollowJointTrajectory,
            self.follow_joint_trajetory_ac_name_,
        )
        while not self.follow_joint_trajetory_ac_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.follow_joint_trajetory_ac_name_} server..."
            )
        self.get_logger().info("Done.")

        # recording client
        self.record_joint_states_client_ = self.create_client(
            SetBool,
            "/joint_state_recording_node/record",
            qos_profile=qos_profile_services_default,
        )
        while not self.record_joint_states_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.record_joint_states_client_} service..."
            )
        self.get_logger().info("Done.")

        # saving client
        self.saving_joint_states_client_ = self.create_client(
            Save,
            "/joint_state_recording_node/save",
            qos_profile=qos_profile_services_default,
        )
        while not self.saving_joint_states_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for {self.saving_joint_states_client_.srv_name} service..."
            )
        self.get_logger().info("Done.")

    def record_(self, record: bool) -> bool:
        self.get_logger().info(f"Setting record to {record}.")

        # send recording request
        record_request = SetBool.Request(data=record)
        record_future = self.record_joint_states_client_.call_async(record_request)
        rclpy.spin_until_future_complete(self, record_future)
        return record_future.result().success

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
        # record
        if not self.record_(True):
            raise RuntimeError(
                f"Failed to start recording {self.record_joint_states_client_.srv_name}"
            )

        # execute
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

        # stop recording
        if not self.record_(False):
            raise RuntimeError(
                f"Failed to stop recording {self.record_joint_states_client_.srv_name}"
            )

    def save(self, path: str, file_name: str) -> bool:
        # stop recording
        if not self.record_(False):
            raise RuntimeError(
                f"Failed to stop recording {self.record_joint_states_client_.srv_name}"
            )

        saving_request = Save.Request(path=path, file_name=file_name)
        saving_future = self.saving_joint_states_client_.call_async(saving_request)
        rclpy.spin_until_future_complete(self, saving_future)


import csv
import os

from ament_index_python import get_package_share_directory


def main(args: List = None) -> None:
    rclpy.init(args=args)
    trajectory_execution_node = TrajectoryExecutionNode()

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

            # go to target position
            trajectory_execution_node.go_to(
                joint_names=joint_names,
                target_joint_position=target_positions,
                time_from_start=Duration(sec=4, nanosec=0),
            )

    # save results
    trajectory_execution_node.save(path="/tmp", file_name="joint_states.csv")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
