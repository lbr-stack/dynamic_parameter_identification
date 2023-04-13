from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger


class JointStateRecordingNode(Node):
    def __init__(self, node_name: str = "joint_state_recording_node") -> None:
        super().__init__(node_name=node_name)
        self.joint_state_ = None
        self.positions_, self.velocities_, self.efforts_ = [], [], []
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
            Trigger,
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
            self.positions_.append(position)
            self.velocities_.append(velocity)
            self.efforts_.append(effort)

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
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("ATTEMPTING TO SAVE")

        # save data to csv

        self.clear_records_()
        return response


def main(args: List = None) -> None:
    rclpy.init(args=args)
    joint_state_recording_node = JointStateRecordingNode()
    rclpy.spin(joint_state_recording_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
