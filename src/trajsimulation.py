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








class TrajectoryConductionSim(Node):
    def __init__(self,file_name,resource_list,is_traj_from_path=True,traj_data="/home/thy/target_joint_states.csv",gravity_vector=[0, 0, -9.81]) -> None:
        

        # resource_path1 = os.path.join(
        #             get_package_share_directory("med7_dock_description")
        #         )
        # resource_path2 = os.path.join(
        #             get_package_share_directory("lbr_description")
        #         )
        # print("RUn to here")
        p.connect(p.DIRECT)
        # p.connect(p.GUI)
        p.setTimeStep(0.01)
        p.setGravity(*gravity_vector)
        for path in resource_list:
            p.setAdditionalSearchPath(path)
        # p.setAdditionalSearchPath(resource_path2)
        robot_id = p.loadURDF(file_name, useFixedBase=True)
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

    def __del__(self):
        p.disconnect()


    
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

    def run_sim_to_list(self):
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
            joint_forces = [-state[3] for state in joint_states]
            data.append(joint_positions + joint_forces)
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
            joint_forces = [-state[3] for state in joint_states]
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
    file_name = "med7dock.urdf"
    paths = [
        os.path.join(
            get_package_share_directory("med7_dock_description")
        ),
        os.path.join(
            get_package_share_directory("lbr_description")
        )
    ]
    instance = TrajectoryConductionSim(file_name, paths)
    instance.set_friction_params()
    instance.run_sim()
    # instance.set_gravity_vector([4.905, 0.0, -8.496])
    # instance.run_sim()
    rclpy.shutdown()
    


        

if __name__ == "__main__":
    main()


# resource_path1 = os.path.join(
#             get_package_share_directory("med7_dock_description")
#         )
# resource_path2 = os.path.join(
#             get_package_share_directory("lbr_description")
#         )



# # 初始化PyBullet
# p.connect(p.GUI)
# p.setGravity(0, 0, -9.81)
# p.setAdditionalSearchPath(resource_path1)
# p.setAdditionalSearchPath(resource_path2)
# p.setTimeStep(0.01)

# # 加载机械臂模型（以UR5为例）
# xacro_filename = os.path.join(
#             get_package_share_directory("med7_dock_description"),
#             "urdf",
#             "med7dock.urdf.xacro",
#         )

# print("path = ",resource_path2)


# urdf_string = xacro.process(xacro_filename)




# # 定义替换规则
# package_path = "package://lbr_description"
# actual_path = resource_path2

# # 使用str.replace()替换路径
# urdf_string = urdf_string.replace(package_path, actual_path)



# package_path = "package://med7_dock_description"
# actual_path = resource_path1

# # 使用str.replace()替换路径
# urdf_string = urdf_string.replace(package_path, actual_path)




# # 定义要保存的文件名
# file_name = "med7dock.urdf"

# # # 使用with语句打开文件，确保文件正确关闭
# # with open(file_name, "w") as file:
# #     file.write(urdf_string)

# # print(f"URDF file saved as {file_name}")


# robot_id = p.loadURDF(file_name, useFixedBase=True)
# # 用于存储数据的列表
# data = []


# for joint_index in range(7):
#     p.enableJointForceTorqueSensor(robot_id, joint_index, enableSensor=1)

# non_fixed_joints = []
# for joint_index in range(p.getNumJoints(robot_id)):
#     joint_info = p.getJointInfo(robot_id, joint_index)
#     joint_type = joint_info[2]
#     if joint_type != p.JOINT_FIXED:
#         non_fixed_joints += [joint_index]



# qs_des_ = []
# with open(
#     "/home/thy/target_joint_states.csv", newline=""
# ) as csvfile:
#     csv_reader = csv.DictReader(csvfile)

#     # read csv and fill self.qs_des_
#     for row in csv_reader:
#         qs_des_.append(
#             [float(qi_des) for qi_des in list(row.values())[1:8]]
#         )



# # p.enableJointForceTorqueSensor(robot_id)
# for index, joint in enumerate(non_fixed_joints):
#     p.resetJointState(robot_id, joint, qs_des_[0][index])
# print("non_fixed_joints = ",non_fixed_joints)

# # 模拟一系列位置控制
# for step in range(len(qs_des_)):
#     # 示例：随机设置关节目标位置
#     # print("qs_des_")
#     target_positions = qs_des_[step]
#     for index, joint in enumerate(non_fixed_joints):
#         # print("joint = ",joint)
#         p.setJointMotorControl2(robot_id, joint, p.POSITION_CONTROL, target_positions[index])

#     # 执行一步仿真
#     p.stepSimulation()

#     # 收集并记录关节的位置和力
#     joint_states = p.getJointStates(robot_id, non_fixed_joints)
#     joint_positions = [state[0] for state in joint_states]
#     joint_forces = [state[3] for state in joint_states]
#     data.append(joint_positions + joint_forces)
#     print("joint_forces = ",joint_forces)

#     # 等待一小段时间（根据实际情况调整）
#     time.sleep(1. / 100.)

# # 断开PyBullet
# p.disconnect()

# # 保存数据到CSV文件
# with open('robot_data.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(['Joint1_Pos', 'Joint2_Pos', 'Joint3_Pos', 'Joint4_Pos', 'Joint5_Pos', 'Joint6_Pos', 'Joint7_Pos', 
#                      'Joint1_Force', 'Joint2_Force', 'Joint3_Force', 'Joint4_Force', 'Joint5_Force', 'Joint6_Force','Joint7_Force'])
#     writer.writerows(data)