#!/usr/bin/python3
import os
import time

import kinpy
import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=1000)
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import xacro
from ament_index_python import get_package_share_directory
from rclpy import qos
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from lbr_fri_msgs.msg import LBRCommand, LBRState
from std_msgs.msg import Float64MultiArray

import pathlib
import csv
import copy
# OpTaS
import optas
from optas.spatialmath import *

from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
import math



from dynamicLin import RNEA_function,DynamicLinearlization,getJointParametersfromURDF,find_dyn_parm_deps,ExtractFromParamsCsv

# import csv


# ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=false controller_configurations_package:=lbr_velocity_controllers controller_configurations:=config/sample_config.yml controller:=lbr_velocity_controller 




class AdmittanceControlNode(Node):
    def __init__(self, node_name="admittance_control_node") -> None:
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("model", "med7dock")
        self.declare_parameter("end_link_name", "link_shaft")
        self.declare_parameter("root_link_name", "lbr_link_0")
        self.declare_parameter("buffer_len", 40)

        self.model_ = str(self.get_parameter("model").value)
        self.end_link_name_ = str(self.get_parameter("end_link_name").value)
        self.root_link_name_ = str(self.get_parameter("root_link_name").value)

        # controller
        path = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            f"{self.model_}.urdf.xacro",
        )
        self.urdf_string_ = xacro.process(path)

        self.chain_ = kinpy.build_serial_chain_from_urdf(
            data=self.urdf_string_, end_link_name=self.end_link_name_, root_link_name=self.root_link_name_
        )


        self.robot = optas.RobotModel(
            xacro_filename=path,
            time_derivs=[1],  # i.e. joint velocity
        )
        Nb, xyzs, rpys, axes = getJointParametersfromURDF(self.robot)
        self.dynamics_ = RNEA_function(Nb,1,rpys,xyzs,axes)
        self.Ymat, self.PIvector = DynamicLinearlization(self.dynamics_,Nb)

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        self.Pb = Pb
        self.K = K

        path_pos = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "DynamicParameters.csv",
        )
        self.params = ExtractFromParamsCsv(path_pos)


        self.lbr_command_ = LBRCommand()
        self.lbr_state_ = None

        # publishers and subscribers
        cb_group = ReentrantCallbackGroup()
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr/state", self.lbr_state_cb_,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
            callback_group=cb_group
        )

        self.cam_state_sub_ = self.create_subscription(
            Pose, 'object_pose_topic',self.cam_state_cb_, qos.qos_profile_system_default)

        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand, "/lbr/command",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )

        self.timer_ = self.create_timer(
            0.02, self.on_timer_, callback_group=cb_group
        )

        self.init_ = True
        self.command_init_ = False
        self.goal_pose_ = None
        self.qd = np.array([0.0]*7)
        self.qd_last = np.array([0.0]*7)
        self.pa_size = Pb.shape[1]

        self.T_bt = np.zeros([4,4])
        self.T_bt[3,3] = 1.0
        self.tau_ext_ground = None
        self.force_filter = TD_list_filter(T=0.01)
        self.dq = np.array([0.0]*7)

        # pg = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def on_timer_(self):
        if not self.lbr_state_:
            return
        q = np.array(self.lbr_state_.measured_joint_position.tolist())
        tau = np.array(self.lbr_state_.measured_torque.tolist())

        qdd = (self.qd-self.qd_last)/0.01
        current_pose = self.chain_.forward_kinematics(q)


        tau_ext = tau - (self.Ymat(q.tolist(),
                                   self.qd.tolist(),
                                   qdd.tolist())@self.Pb @  self.params[:self.pa_size] + 
                np.diag(np.sign(self.qd)) @ self.params[self.pa_size:self.pa_size+7]+ 
                np.diag(qdd) @ self.params[self.pa_size+7:])


        self.jacobian_ = np.array(self.chain_.jacobian(q))
        self.jacobian_inv_ = np.linalg.pinv(self.jacobian_, rcond=0.05)
        
        dq = 

        self.dq = dq
        # command
        
        # print("self.lbr_state_.sample_time * dq * 1.0 = {0}",format(self.lbr_state_.sample_time * dq * 1.0))
        # print("goal = {0}",format(self.goal_pose_))

        self.command_init_ = True

    def lbr_state_cb_(self, msg: LBRState) -> None:
        self.lbr_state_ = msg

        self.lbr_command_.joint_position = (
            np.array(self.lbr_state_.measured_joint_position.tolist())
            + self.lbr_state_.sample_time * self.dq * 1.0
        ).data

        if self.command_init_:
            self.lbr_command_pub_.publish(self.lbr_command_)

    def cam_state_cb_(self, msg:Pose) ->None:
        self.goal_pose_ = msg
        # print()

def main(args=None):
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
