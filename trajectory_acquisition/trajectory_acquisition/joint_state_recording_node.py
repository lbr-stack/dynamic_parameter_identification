import csv
import os
from typing import List

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from trajectory_acquisition_msgs.srv import Save


class JointStateRecordingNode(Node):
    def __init__(self, node_name: str = "joint_state_recording_node") -> None:
        super().__init__(node_name=node_name)
        self.joint_state_ = None
        self.positions_, self.velocities_, self.efforts_, self.time_stamp_ = (
            [],
            [],
            [],
            [],
        )
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_sub_sb_,
            qos_profile_sensor_data,
        )

        self.recording_ = False
        self.recording_service_ = self.create_service(
            SetBool,
            "~/record",
            self.recording_service_cb_,
            qos_profile=qos_profile_services_default,
        )
        self.save_record_trigger_service_ = self.create_service(
            Save,
            "~/save",
            self.save_record_trigger_service_cb_,
            qos_profile=qos_profile_services_default,
        )

    def joint_state_sub_sb_(self, joint_state: JointState) -> None:
        self.joint_state_ = joint_state
        if self.joint_state_ and self.recording_:
            position, velocity, effort = (
                self.joint_state_.position,
                self.joint_state_.velocity,
                self.joint_state_.effort,
            )
            time_stamp = self.time_stamp_to_sec_(self.joint_state_.header.stamp)
            self.positions_.append(position)
            self.velocities_.append(velocity)
            self.efforts_.append(effort)
            self.time_stamp_.append(time_stamp)

    def time_stamp_to_sec_(self, time_stamp: Time) -> float:
        return float(time_stamp.sec) + float(time_stamp.nanosec) / 1.0e9

    def recording_service_cb_(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.recording_ = request.data
        response.success = self.recording_ == request.data
        response.message = f"Set recording to {self.recording_}"
        self.get_logger().info(response.message)
        return response

    def clear_records_(self) -> None:
        self.positions_.clear()
        self.velocities_.clear()
        self.efforts_.clear()

    def save_record_trigger_service_cb_(
        self, request: Save.Request, response: Save.Response
    ) -> Save.Response:
        file_name, postfix = request.file_name.split(".")
        self.save_(
            request.path,
            file_name + "_positions." + postfix,
            ["time"] + self.joint_state_.name,
            [
                [time] + list(positions)
                for time, positions in zip(self.time_stamp_, self.positions_)
            ],
        )
        self.save_(
            request.path,
            file_name + "_velocities." + postfix,
            ["time"] + self.joint_state_.name,
            [
                [time] + list(velocities)
                for time, velocities in zip(self.time_stamp_, self.velocities_)
            ],
        )
        self.save_(
            request.path,
            file_name + "_efforts." + postfix,
            ["time"] + self.joint_state_.name,
            [
                [time] + list(efforts)
                for time, efforts in zip(self.time_stamp_, self.efforts_)
            ],
        )
        self.get_logger().info("Done.")
        self.clear_records_()
        response.message = f"Wrote files to {request.path}"
        response.success = True
        return response

    def save_(
        self, path: str, file_name: str, keys: List[str], values_list: List[List[float]]
    ) -> None:
        # save data to csv
        full_path = os.path.join(path, file_name)
        self.get_logger().info(f"Saving to {full_path}...")
        with open(full_path, "w") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=keys)

            csv_writer.writeheader()
            for values in values_list:
                csv_writer.writerow({key: value for key, value in zip(keys, values)})


def main(args: List = None) -> None:
    rclpy.init(args=args)
    joint_state_recording_node = JointStateRecordingNode()
    rclpy.spin(joint_state_recording_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
