#!/usr/bin/python3
import pybullet as p
import pybullet_data
import time
import csv
import os
import xacro
from ament_index_python import get_package_share_directory
import rclpy
from rclpy import qos
from rclpy.node import Node
import numpy as np


import xml.etree.ElementTree as ET


import xml.etree.ElementTree as ET

import rospkg
import xml.etree.ElementTree as ET

import optas
from scipy.spatial.transform import Rotation as Rot
from optas.spatialmath import *
from IDmodel import TD_2order, TD_list_filter,find_dyn_parm_deps, RNEA_function,DynamicLinearlization,getJointParametersfromURDF
import urdf_parser_py.urdf as urdf

def get_axis_torque(axis, torques):
    # 轴的方向
    ax, ay, az = axis
    # 对应的力矩值
    mx, my, mz = torques
    
    # 根据轴方向获取力矩值
    if ax != 0:
        return mx if ax > 0 else -mx
    elif ay != 0:
        return my if ay > 0 else -my
    elif az != 0:
        return mz if az > 0 else -mz
    else:
        return 0  # 如果轴方向为 [0, 0, 0]，返回 0
    
def calculate_joint_accelerations(joint_velocities_1, joint_velocities_2, delta_time):
    joint_accelerations = [(v2 - v1) / delta_time for v1, v2 in zip(joint_velocities_1, joint_velocities_2)]
    return joint_accelerations
    
def resolve_package_path(package_path):
    """
    Resolves the absolute path of a ROS package.
    :param package_path: The package path starting with 'package://'
    :return: The absolute path of the package or raises an exception if not found
    """
    if not package_path.startswith('package://'):
        return package_path

    # Remove the 'package://' prefix
    package_path = package_path[len('package://'):]
    
    # Extract the package name and the relative path inside the package
    package_name, relative_path = package_path.split('/', 1)
    
    # Use ament_index_python to get the package path
    try:
        package_dir = get_package_share_directory(package_name)
    except KeyError:
        raise Exception(f"Package '{package_name}' not found. Make sure the package is installed and sourced properly.")
    
    # Construct the absolute path
    absolute_path = os.path.join(package_dir, relative_path)
    return absolute_path

def replace_package_paths_in_xacro(xacro_file):
    """
    Replaces 'package://' paths with absolute paths in the given Xacro file.
    :param xacro_file: Path to the input Xacro file.
    """
    tree = ET.parse(xacro_file)
    # print("xacro_file = ",xacro_file)
    root = tree.getroot()

    for mesh in root.findall('.//mesh'):
        filename = mesh.get('filename')
        if filename and filename.startswith('package://'):
            try:
                # Resolve the absolute path
                new_filename = resolve_package_path(filename)
                mesh.set('filename', new_filename)
                print(f'Replaced: {filename} with {new_filename}')
            except Exception as e:
                print(e)
    
    # Write the modified XML back to the file
    modified_xacro_file = xacro_file
    tree.write(modified_xacro_file)
    print(f'Modified Xacro file saved as: {modified_xacro_file}')


class TrajectoryConductionSim(Node):
    def __init__(self,
                file_name,
                resource_list,
                is_traj_from_path=True,
                traj_data="/home/thy/target_joint_states.csv",
                gravity_vector=[0, 0, -9.81],
                use_gui = True
                ) -> None:
        

        # resource_path1 = os.path.join(
        #             get_package_share_directory("med7_dock_description")
        #         )
        # resource_path2 = os.path.join(
        #             get_package_share_directory("lbr_description")
        #         )
        # print("RUn to here")
        if use_gui == False:
            p.connect(p.DIRECT)
        else:
        # global p
            p.connect(p.GUI)
        p.setTimeStep(0.01)
        p.setGravity(*gravity_vector)
        for path in resource_list:
            p.setAdditionalSearchPath(path)
            print("path = ",path)
        # p.setAdditionalSearchPath(resource_path2)


        xacro_file = file_name
        replace_package_paths_in_xacro(xacro_file)
        # time.sleep(1)
        print("run to here")
        # file_name = 'med7dock.urdf'
        print("file_name =",file_name)
        robot_id = p.loadURDF(file_name, useFixedBase=True)
        print("run to here")

        for joint_index in range(p.getNumJoints(robot_id)):
            p.enableJointForceTorqueSensor(robot_id, joint_index, enableSensor=1)

        self.robot_id = robot_id


        non_fixed_joints = []
        for joint_index in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                non_fixed_joints += [joint_index]
        self.non_fixed_joints = non_fixed_joints

        if(traj_data is not None):
            if(is_traj_from_path):
                self.import_traj_frompath(traj_data)
            else:
                self.import_traj_fromlist(traj_data)

        self.robot = optas.RobotModel(
            xacro_filename=file_name,
            time_derivs=[1],  # i.e. joint velocity
        )
        Nb, xyzs, rpys, axes = getJointParametersfromURDF(self.robot)
        self.dynamics_ = RNEA_function(Nb,1,rpys,xyzs,axes,gravity_para = cs.DM(gravity_vector))
        self.Ymat, self.PIvector = DynamicLinearlization(self.dynamics_,Nb)


        urdf_string_ = xacro.process(file_name)
        robot = urdf.URDF.from_xml_string(urdf_string_)

        masses = [link.inertial.mass for link in robot.links if link.inertial is not None]#+[1.0]
        self.masses_np = np.array(masses[1:])
        massesCenter = [link.inertial.origin.xyz for link in robot.links if link.inertial is not None]#+[[0.0,0.0,0.0]]
        self.massesCenter_np = np.array(massesCenter[1:]).T
        Inertia = [link.inertial.inertia.to_matrix() for link in robot.links if link.inertial is not None]
        self.Inertia_np = np.hstack(tuple(Inertia[1:]))
        self.params = None
        self.Pb = None

    def setup_params_sim(self,params):
        self.params = np.asarray(params)

        _w1, _h1 =self.massesCenter_np.shape
        _w2, _h2 =self.Inertia_np.shape
        _w0 = len(self.masses_np)

        l = _w0 + _h1*_w1 + _w2 * _h2
        l1 = _w0 + _w1*_h1

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T
        self.estimate_gt = K @ self.PIvector(self.params[0:_w0],
                                        self.params[_w0:l1].reshape((_w1,_h1)),
                                        self.params[l1:l].reshape((_w2,_h2)))
        
        self.Pb = Pb

        # print("self.estimate_gt = ",self.estimate_gt.shape)
        # print("self.params 2 = ",self.params[-7:])

        # raise ValueError("1111")


    def inverse_dynamics(self, q_np, qd_np, qdd_np):
        if self.params is None:
            raise ValueError("Forgot Params Setting")
        tau_ext = (self.Ymat(q_np,qd_np,qdd_np) @self.Pb@  self.estimate_gt + 
                np.diag(np.sign(qd_np)) @ self.params[-2*len(qd_np):-len(qd_np)]+ 
                np.diag(qd_np) @ self.params[-len(qd_np):])
        return tau_ext.full().flatten()



    # def __del__(self):
    #     p.disconnect()


    
    def import_traj_frompath(self, path="/home/thy/target_joint_states.csv"):
        self.qs_des_ = []
        with open(
            path, newline=""
        ) as csvfile:
            csv_reader = csv.DictReader(csvfile)

            # read csv and fill self.qs_des_
            for row in csv_reader:
                self.qs_des_.append(
                    [float(qi_des) for qi_des in list(row.values())[1:8]]
                )
    
    def import_traj_fromlist(self, trajlist):
        self.qs_des_ = []
        # with open(
        #     path, newline=""
        # ) as csvfile:
        # csv_reader = csv.DictReader(trajlist)

        # read csv and fill self.qs_des_
        for row in trajlist:
            self.qs_des_.append(
                [float(qi_des) for qi_des in row[1:8]]
            )

    def run_sim_in_workspace(self):
        
        # non_fixed_joints = []
        joint_lower_limit = []
        joint_upper_limit = []

        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                # non_fixed_joints += [joint_index]
                joint_name = joint_info[1].decode('utf-8')
                joint_lower_limit.append(joint_info[8]) 
                joint_upper_limit.append(joint_info[9])  
                # print(f"Joint {joint_name} (index {joint_index}): lower limit = {joint_lower_limit}, upper limit = {joint_upper_limit}")

        # self.non_fixed_joints = non_fixed_joints
        print(joint_upper_limit)
        for index, joint in enumerate(self.non_fixed_joints):
            p.enableJointForceTorqueSensor(self.robot_id, joint, enableSensor=True)

        num_samples = 1000

        # 创建空数组来存储采样点
        samples = np.empty((num_samples, len(joint_lower_limit)))

        # 遍历每个维度，根据上下界进行均匀采样
        data =[]
        for i in range(len(joint_lower_limit)):
            samples[:, i] = np.random.uniform(joint_lower_limit[i], joint_upper_limit[i], num_samples)

        samples_list = samples.tolist()
        joint_vels_last = [0.0]*len(self.non_fixed_joints)

        for step in range(len(samples_list)):
            target_positions = samples_list[step]

            for index, joint in enumerate(self.non_fixed_joints):
                p.resetJointState(self.robot_id, joint, target_positions[index])
            # for index, joint in enumerate(self.non_fixed_joints):
            #     # print("joint = ",joint)
            #     # p.setJointMotorControl2(self.robot_id, joint, p.TORQUE_CONTROL, 0.0)
            #     # p.setJointMotorControl2(self.robot_id, joint, p.POSITION_CONTROL, target_positions[index])
            #     p.setJointMotorControl2(self.robot_id, joint, p.VELOCITY_CONTROL, 0.0)
            # p.stepSimulation()

            # 进行多次仿真步进，让机器人达到稳定状态
            # for index, joint in enumerate(self.non_fixed_joints):
            #     p.resetJointState(self.robot_id, joint, target_positions[index])
            # p.stepSimulation()
            # for _ in range(500):  # 可以调整步数
            #     p.stepSimulation()
            js_infos = []
            for id in self.non_fixed_joints:
                js_info = p.getJointInfo(self.robot_id, id)
                # print ("joint_info[13] = ", js_info[13])
                js_infos.append(js_info[13])
            joint_states = p.getJointStates(self.robot_id, self.non_fixed_joints)
            
            joint_positions = [state[0] for state in joint_states]
            joint_vels = [0.0 for state in joint_states]

            _accelerations = calculate_joint_accelerations(joint_vels, joint_vels_last, 0.01)

            joint_vels_last = joint_vels

            if self.params is None:
                joint_torques = p.calculateInverseDynamics(self.robot_id, joint_positions, joint_vels, _accelerations)
            else:
                joint_torques = self.inverse_dynamics(joint_positions, joint_vels,_accelerations)
            # joint_forces = [-get_axis_torque( js_info,state[2][3:]) for state, js_info in zip(joint_states,js_infos)]

            # print("joint_torques =",joint_torques)
            # print("joint_forces =",joint_forces)

            # raise ValueError("111")
            data.append(joint_positions + list(joint_torques))

        return data

        # print(
        #     "data", data
        # )

        # print("samples = ",samples_list)
        


    def run_sim_to_list(self):
        for index, joint in enumerate(self.non_fixed_joints):
            p.resetJointState(self.robot_id, joint, self.qs_des_[0][index])
        print("non_fixed_joints = ",self.non_fixed_joints)

        for index, joint in enumerate(self.non_fixed_joints):
            p.enableJointForceTorqueSensor(self.robot_id, joint, enableSensor=True)

        # 模拟一系列位置控制
        data =[]
        joint_vels_last = [0.0]*len(self.non_fixed_joints)

        
        for step in range(len(self.qs_des_)):
            # 示例：随机设置关节目标位置
            # print("qs_des_")
            target_positions = self.qs_des_[step]
            for index, joint in enumerate(self.non_fixed_joints):
                p.resetJointState(self.robot_id, joint, target_positions[index])
            # for index, joint in enumerate(self.non_fixed_joints):
            #     p.setJointMotorControl2(self.robot_id, joint, p.POSITION_CONTROL, target_positions[index])

            # 执行一步仿真
            p.stepSimulation()

            # 收集并记录关节的位置和力
            js_infos = []
            for id in self.non_fixed_joints:
                js_info = p.getJointInfo(self.robot_id, id)
                # print ("joint_info[13] = ", js_info[13])
                js_infos.append(js_info[13])
            # raise ValueError("111")

            joint_states = p.getJointStates(self.robot_id, self.non_fixed_joints)
            joint_positions = [state[0] for state in joint_states]
            joint_vels = [state[1] for state in joint_states]


            _accelerations = calculate_joint_accelerations(joint_vels, joint_vels_last, 0.01)
            joint_vels_last = joint_vels


            #TODO
            if self.params is None:
                joint_torques = p.calculateInverseDynamics(self.robot_id, joint_positions, joint_vels, _accelerations)
            else:
                joint_torques = self.inverse_dynamics(joint_positions, joint_vels,_accelerations)
                
            # joint_forces = [-get_axis_torque( js_info,state[2][3:]) for state, js_info in zip(joint_states,js_infos)]
            data.append(joint_positions + list(joint_torques))
            # print("joint_forces = ",joint_forces)

            # 等待一小段时间（根据实际情况调整）
            time.sleep(1. / 100.)
        return data

        # with open(name, 'w', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerow(['Joint1_Pos', 'Joint2_Pos', 'Joint3_Pos', 'Joint4_Pos', 'Joint5_Pos', 'Joint6_Pos', 'Joint7_Pos', 
        #                     'Joint1_Force', 'Joint2_Force', 'Joint3_Force', 'Joint4_Force', 'Joint5_Force', 'Joint6_Force','Joint7_Force'])
        #     writer.writerows(data)



    def run_sim(self, name='robot_data.csv'):
        for index, joint in enumerate(self.non_fixed_joints):
            p.resetJointState(self.robot_id, joint, self.qs_des_[0][index])
        print("non_fixed_joints = ",self.non_fixed_joints)

        # 模拟一系列位置控制
        data =[]
        for step in range(len(self.qs_des_)):
            # 示例：随机设置关节目标位置
            # print("qs_des_")
            target_positions = self.qs_des_[step]
            for index, joint in enumerate(self.non_fixed_joints):
                # print("joint = ",joint)
                p.setJointMotorControl2(self.robot_id, joint, p.POSITION_CONTROL, target_positions[index])

            # 执行一步仿真
            p.stepSimulation()

            # 收集并记录关节的位置和力
            joint_states = p.getJointStates(self.robot_id, self.non_fixed_joints)
            joint_positions = [state[0] for state in joint_states]
            joint_forces = [state[2][5] for state in joint_states]
            data.append(joint_positions + joint_forces)
            print("joint_forces = ",joint_forces)

            # 等待一小段时间（根据实际情况调整）
            time.sleep(1. / 100.)

        with open(name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Joint1_Pos', 'Joint2_Pos', 'Joint3_Pos', 'Joint4_Pos', 'Joint5_Pos', 'Joint6_Pos', 'Joint7_Pos', 
                            'Joint1_Force', 'Joint2_Force', 'Joint3_Force', 'Joint4_Force', 'Joint5_Force', 'Joint6_Force','Joint7_Force'])
            writer.writerows(data)

        # 断开PyBullet
        # p.disconnect()
            

    def set_gravity_vector(self, vector = [0.0, 0.0, -9.81]):
        p.setGravity(*vector)


    def set_friction_params(self, friction_para=0.00):
        for index, joint in enumerate(self.non_fixed_joints):
            out = p.getDynamicsInfo(self.robot_id, joint)
            o = p.getJointInfo(self.robot_id, joint)
            print("out = ",out)
            print("o = ",o)
            p.changeDynamics(self.robot_id, joint, lateralFriction=friction_para)



def generateURDF():
    resource_path1 = os.path.join(
                get_package_share_directory("med7_dock_description")
            )
    resource_path2 = os.path.join(
                get_package_share_directory("lbr_description")
            )
    package_paths = ["package://lbr_description", "package://med7_dock_description","damping=\"10.0\""]
    actual_paths = [resource_path2, resource_path1,"damping=\"0.1\""]
    xacro_filename = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            "med7dock.urdf.xacro",
        )
    
    generateURDFwithReplace(xacro_filename, "med7dock.urdf",package_paths,actual_paths)



def generateURDFwithReplace(xacro_filename, urdf_path, package_list, actual_list):
    urdf_string = xacro.process(xacro_filename)

    # 使用str.replace()替换路径
    for package_path, actual_path in zip(package_list, actual_list):
        urdf_string = urdf_string.replace(package_path, actual_path)

    # 定义要保存的文件名
    # file_name = "med7dock.urdf"

    # 使用with语句打开文件，确保文件正确关闭
    with open(urdf_path, "w") as file:
        file.write(urdf_string)




    # # package_path = "package://med7_dock_description"
    # # actual_path = resource_path1
    # urdf_string = urdf_string.replace(package_path, actual_path)



    # # package_path = "damping=\"10.0\""
    # # actual_path = "damping=\"0.1\""
    # urdf_string = urdf_string.replace(package_path, actual_path)






            


def main(args=None):
    rclpy.init(args=args)
    file_name = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "urdf",
            "med",
            "med7dock.urdf"
        )
    paths = [
        os.path.join(
            get_package_share_directory("med7_dock_description")
        ),
        os.path.join(
            get_package_share_directory("lbr_description")
        ),
        os.path.join(
            get_package_share_directory("gravity_compensation"),
            "urdf",
            "med"
        )
    ]
    instance = TrajectoryConductionSim(file_name, paths,traj_data = '/tmp/target_joint_states.csv')
    instance.set_friction_params()
    instance.run_sim()
    # instance.set_gravity_vector([4.905, 0.0, -8.496])
    # instance.run_sim()
    rclpy.shutdown()
    


        

if __name__ == "__main__":
    main()

