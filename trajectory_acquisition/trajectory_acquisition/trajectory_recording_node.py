
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

        self.joint_state_ = None
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_sub_sb_,
            qos_profile_sensor_data,
        )




