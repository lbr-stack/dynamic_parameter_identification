#!/usr/bin/python3
import os

import numpy as np
np.set_printoptions(precision=3, suppress=True, linewidth=1000)
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import xacro
from ament_index_python import get_package_share_directory

from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Float64MultiArray
from lbr_fri_msgs.msg import LBRCommand, LBRState

import pathlib
import csv
import copy
# OpTaS
import optas
from optas.spatialmath import *

from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
import math
import sys
# sys.path.append(os.path.join(
#             get_package_share_directory("lbr_fri_aruco_detector"),
#             "..",
#             "lib",
#             "lbr_fri_aruco_detector"
#         ))

from IDmodel import RNEA_function,DynamicLinearlization,getJointParametersfromURDF,find_dyn_parm_deps,ExtractFromParamsCsv
sys.path.append("/home/thy/ros2_ws/src/lbr_fri_aruco_detector/src")
from utility_math import csv_save
# import csv


# ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=false controller_configurations_package:=lbr_velocity_controllers controller_configurations:=config/sample_config.yml controller:=lbr_velocity_controller 



# import numpy as np
# import optas

# from lbr_fri_msgs.msg import LBRCommand, LBRState


class AdmittanceController(object):
    def __init__(
        self,
        robot_description: str,
        base_link: str = "lbr_link_0",
        end_effector_link: str = "lbr_link_ee",
        f_ext_th: np.ndarray = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        dq_gain: np.ndarray = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]),
        dx_gain: np.ndarray = np.array([0.2, 0.2, 0.2, 10.0, 10.0, 10.0]),
    ) -> None:
        self.lbr_command_ = LBRCommand()
        # self.declare_parameter("model", "med7dock")
        # print("robot_description",robot_description)

        robot = optas.RobotModel(xacro_filename=robot_description)
        J = robot.get_geometric_jacobian_function(
            end_effector_link, base_link, numpy_output=True
        )
        self.jacobian_func_ = lambda q: J(q)

        self.dof_ = robot.ndof
        self.jacobian_ = np.zeros((6, self.dof_))
        self.jacobian_inv_ = np.zeros((self.dof_, 6))
        self.q = np.zeros(self.dof_)
        self.dq_ = np.zeros(self.dof_)

        self.tau_ext_ = np.array([0.0]*7)
        self.dq_gain_ = np.diag(dq_gain)
        self.dx_gain_ = np.diag(dx_gain)
        self.f_ext_ = np.zeros(6)
        self.f_ext_th_ = f_ext_th
        self.alpha_ = 0.95
        self.q_last = np.array([0.0]*7)#np.zeros(7)
        self.qd_last = np.array([0.0]*7)
        self.qd = np.array([0.0]*7)

        # self.model_ = str(self.get_parameter("model").value)

        # path = os.path.join(
        #     get_package_share_directory("med7_dock_description"),
        #     "urdf",
        #     f"{self.model_}.urdf.xacro",
        # )

        self.robot = robot
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
        self.pa_size = Pb.shape[1]
        self._is_init = False
        self.tau_model = np.array([0.0]*7)
        self.tau_model_th_ = np.array([90.0, 90.0, 90.0, 70.0, 70.0, 70.0,50.0])

    def __call__(self, lbr_state: LBRState) -> LBRCommand:
        self.q_ = np.array(lbr_state.measured_joint_position.tolist())
        self.tau_ext_raw = np.array(lbr_state.measured_torque.tolist())

        # print("lbr_state.measured_joint_position.tolist()", len(lbr_state.measured_joint_position.tolist()))

        if not self._is_init:
            self.q_last = copy.deepcopy(self.q_)
            # c = 
            self._is_init = True

            self.lbr_command_.joint_position = (
                np.array(lbr_state.measured_joint_position.tolist())
            ).data

            return self.lbr_command_

        q = copy.deepcopy(self.q_)
        self.qd = (self.q_-self.q_last)/0.01
        qdd = (self.qd-self.qd_last)/0.01

        # print("q = ",self.q_)
        # print("self.q_", self.q_)
        # print("self.tau_ext_raw", self.tau_ext_raw)


        # self.tau_ext_ =  self.tau_ext_raw 
        self.tau_model = (self.Ymat(q.tolist(),
                        self.qd.tolist(),
                        qdd.tolist())@self.Pb @  self.params[:self.pa_size] + 
                np.diag(np.sign(self.qd)) @ self.params[self.pa_size:self.pa_size+7]+ 
                np.diag(self.qd) @ self.params[self.pa_size+7:]).toarray().flatten()
        

        self.tau_model = np.where(
            abs(self.tau_model) < self.tau_model_th_,
            self.tau_model,
            np.sign(self.tau_model)*self.tau_model_th_,
        )
        

        self.jacobian_ = self.jacobian_func_(self.q_)

        self.jacobian_inv_ = np.linalg.pinv(self.jacobian_, rcond=0.05)
        self.tau_ext_ = self.tau_ext_raw - self.tau_model  #.toarray().flatten()


        self.f_ext_ = self.jacobian_inv_.T @ self.tau_ext_

        # print("self.f_ext_ = ",self.f_ext_)
        print("tau_ext_", self.tau_ext_)

        self.f_ext_ = np.where(
            abs(self.f_ext_) > self.f_ext_th_,
            self.dx_gain_ @ np.sign(self.f_ext_) * (abs(self.f_ext_) - self.f_ext_th_),
            0.0,
        )
        
        self.dq_ = (
            self.alpha_ * self.dq_
            + (1 - self.alpha_) * self.dq_gain_ @ self.jacobian_inv_ @ self.f_ext_
        )




        # self.dq_ = 50*np.ones(7)

        self.lbr_command_.joint_position = (
            np.array(lbr_state.measured_joint_position.tolist())
            + lbr_state.sample_time * self.dq_
        ).data

        

        data_record = np.array(self.tau_model.tolist()+self.dq_.tolist() +self.q_.tolist())
        # csv_save("/home/thy/ros2_ws/tau_model.csv", self.tau_model)
        # csv_save("/home/thy/ros2_ws/tau_ext_raw.csv", self.tau_ext_raw)
        # # csv_save("/home/thy/ros2_ws/f_ext_c.csv", self.f_ext_)
        # csv_save("/home/thy/ros2_ws/data_record.csv", data_record)

        self.qd_last = copy.deepcopy(self.qd)
        self.q_last = copy.deepcopy(self.q_)

      

        return self.lbr_command_



class AdmittanceControlNode(Node):
    def __init__(self, node_name="admittance_control_node") -> None:
        super().__init__(node_name=node_name)

        path = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            "med7dock.urdf.xacro",
        )
        print("path = ",path)

        # parameters
        self.declare_parameter("robot_description", path)
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("end_effector_link", "lbr_link_ee")

        self.init_ = False
        self.lbr_state_ = None

        self.controller_ = AdmittanceController(
            robot_description=path,
            base_link=str(self.get_parameter("base_link").value),
            end_effector_link=str(self.get_parameter("end_effector_link").value),
        )



        # publishers and subscribers
        # cb_group = ReentrantCallbackGroup()
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr/state", self.on_lbr_state_,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand,
            "/lbr/command",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )

    def on_lbr_state_(self, lbr_state: LBRState) -> None:
        self.smooth_lbr_state_(lbr_state, 0.95)
        # print("self.lbr_state_ = ",lbr_state.measured_joint_position.tolist())

        lbr_command = self.controller_(self.lbr_state_)
        # lbr_command = LBRCommand()
        
        self.lbr_command_pub_.publish(lbr_command)

    def smooth_lbr_state_(self, lbr_state: LBRState, alpha: float):
        # pos_list = lbr_state.measured_joint_position.tolist()
        # print("lbr_state.measured_joint_position.tolist()", lbr_state.measured_joint_position.tolist())
        if not self.init_:
            self.lbr_state_ = lbr_state
            self.init_ = True
            return

        self.lbr_state_.measured_joint_position = (
            (1.0 - alpha) * np.array(self.lbr_state_.measured_joint_position.tolist())
            + alpha * np.array(lbr_state.measured_joint_position.tolist())
        ).data

        self.lbr_state_.external_torque = (
            (1.0 - alpha) * np.array(self.lbr_state_.external_torque.tolist())
            + alpha * np.array(lbr_state.external_torque.tolist())
        ).data

        self.lbr_state_.measured_torque = (
            (1.0 - alpha) * np.array(self.lbr_state_.measured_torque.tolist())
            + alpha * np.array(lbr_state.measured_torque.tolist())
        ).data

        # print("self.lbr_state_.measured_joint_position",self.lbr_state_.measured_joint_position)




def main(args=None):
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
