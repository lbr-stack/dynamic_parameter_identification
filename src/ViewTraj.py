import csv
import matplotlib.pyplot as plt

import sys
import numpy as np
from itertools import chain
from mpl_toolkits import mplot3d
import optas
import os
import xacro
from ament_index_python import get_package_share_directory

# sys.path.append('/home/thy/kalmanfilters')

# from KF_learn import load_measurements
mark_list = ['', '', '', 'D', '*','o','D','*']

def load_measurements(path):
    data = []
    with open(path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(row)

    poses = [[float(row[i]) for i in range(len(row))] for row in data[1:]]
    return poses


def view_states_in_one(measuements, N):
    
    # fig, axs = plt.subplots(N, 1, figsize=(8, 10))
    fig, ax = plt.subplots()
    ss = [[] for i in range(N+1)]
    
    for mes in measuements:
        for i in range(N+1):
            ss[i].append(mes[i])
    for i in range(1,N+1):
            ax.plot(ss[0],ss[i], 
                    marker = mark_list[i],
                    label = 'A{0}'.format(i),
            markevery=100)
        # axs[i].set_ylabel('Column {0}'.format(i))
    ax.set_title('Exciting Trajectories')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('Joint Position (rad)')

    ax.legend()
    # 显示图表
    plt.show()


# def view_states_in_one(measuements, N):
    
#     # fig, axs = plt.subplots(N, 1, figsize=(8, 10))
#     fig, ax = plt.subplots()
#     ss = [[] for i in range(N+1)]

#     pos = [[] for i in range(N+1)]
#     base_link = "lbr_link_0"
#     end_effector_link = "lbr_link_ee"
#     path = os.path.join(
#             get_package_share_directory("med7_dock_description"),
#             "urdf",
#             "med7dock.urdf.xacro",
#         )

#     robot = optas.RobotModel(urdf_string=path)
    
#     for mes in measuements:
#         for i in range(N+1):
#             ss[i].append(mes[i])
#         J = robot.get_geometric_jacobian_function(
#             end_effector_link, base_link, numpy_output=True
#         )
#         pos[0].append(x)
#         pos[1].append(y)
#         pos[2].append(z)
#     # for i in range(1,N+1):
#     ax.plot(pos[0],pos[1],pos[2], marker = mark_list[3],
#             markevery=100)
#         # axs[i].set_ylabel('Column {0}'.format(i))
#     ax.set_title('Exciting Trajectories')
#     ax.set_xlabel('time (s)')
#     ax.set_ylabel('Joint Position (rad)')

#     ax.legend()
#     # 显示图表
#     plt.show()

def view_Cartesian_pose(measuements, N):
    
    # fig, axs = plt.subplots(N, 1, figsize=(8, 10))
    # fig, ax = plt.subplots()
    ss = [[] for i in range(N+1)]
    pos = [[] for i in range(N+1)]
    base_link = "lbr_link_0"
    end_effector_link = "lbr_link_ee"
    path = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            "med7dock.urdf.xacro",
        )
    ax = plt.figure().add_subplot(projection='3d')

    robot = optas.RobotModel(xacro_filename=path)
    

    
    for mes in measuements:
        for i in range(N+1):
            ss[i].append(mes[i])

        x,y,z = robot.get_global_link_position(
            end_effector_link, np.asarray(mes[1:8])
        ).toarray().flatten()

        pos[0].append(x)
        pos[1].append(y)
        pos[2].append(z)

    
    # for i in range(1,N+1):
    ax.plot(pos[0],pos[1],pos[2], marker = mark_list[4],
            markevery=100)
        # axs[i].set_ylabel('Column {0}'.format(i))
    ax.set_title('Exciting Trajectories')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('Joint Position (rad)')

    ax.legend()
    # 显示图表
    plt.show()

def quaternion_to_rotation_matrix(q):

    w, x, y, z = q
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    return R

# sys.path.append('/home/thy/ros2_ws/src/lbr_fri_ros2_stack/lbr_demos/lbr_fri_ros2_advanced_python_demos/lbr_fri_ros2_advanced_python_demos')
# from admittance_control_Hybrid import quaternion_to_rotation_matrix
# 读取CSV文件
#bt和br都是世界坐标系下的，因此他们不相等其实并不影响。因为姿态并非严格为100
# id_str = "51"
# filename = id_str+'/pose_bt.csv'
# filename2 = id_str+'/pose_br.csv'/home/thy/target traj
filename3 = "/home/thy/target traj/19/target_joint_states.csv"


fs = load_measurements(filename3)
# view_states(fs,7)
view_states_in_one(fs, 7)
view_Cartesian_pose(fs, 7)
