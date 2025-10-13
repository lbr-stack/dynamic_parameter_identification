#!/usr/bin/python3
import optas
import sys
import numpy as np
import os

import rclpy
import xacro
from ament_index_python import get_package_share_directory
from rclpy import qos
from rclpy.node import Node
import csv

from TrajGeneration import TrajGeneration

from trajsimulation import TrajectoryConductionSim
from regression import Estimator,traj_filter,compare_traj
from utility_math import csv_save

from TrajGeneration import TrajGenerationUsrPath
from trajsimulation import TrajectoryConductionSim
import matplotlib.pyplot as plt


sys.path.append('/home/thy/learningDynModel')
# import torch
from er_app import er_state_compensator


def generate_excitation_sat_path(robot_urdf:str,
                                 gravity_vec:list,
                                 Ff:float = 0.1,
                                 inopt_rate:float = 20.0,
                                 output_rate:float = 100.0,
                                 cond_th:float=400.0,
                                 theta1:float=0.0,
                                 theta2:float=0.0):
    conditional_num = 1e12
    traj_instance = TrajGenerationUsrPath(path=robot_urdf, gravity_vector=gravity_vec)
    S,lbg,ubg,fc = traj_instance.get_optimization_problem(Ff = Ff,sampling_rate = inopt_rate, bias = [theta1, theta2, 0.0, 0.0, 0.0, 0.0, 0.0])
    while(conditional_num>cond_th):
        a,b = traj_instance.find_optimal_point_with_randomstart(S,lbg,ubg, Rank=5)
        # print("a = {0} \n b = {1}".format(a,b))
        eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))
        conditional_num = np.sqrt(eigenvalues[0]/eigenvalues[-1])
        values_list,keys = traj_instance.generateToList(a,b,Ff = Ff,sampling_rate=output_rate)

        print("conditional_num = ",conditional_num)

    return values_list,conditional_num
def process_regression_data(data, prefix, filename, paraEstimator, traj_type='traj'):
    if traj_type == 'traj':
        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentList(data)
    elif traj_type == 'MC_test':
        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentListZeroVel(data)
    else:
        raise ValueError("Not define the type" + traj_type)
    efforts_f = efforts
    # 可以根据需要取消注释以滤除噪音
    # velocities = traj_filter(velocities)
    # efforts_f = traj_filter(efforts)
    ref_pam = paraEstimator.get_gt_params_simO()
    
    estimate_pam, ref_pam = paraEstimator.timer_cb_regressor_physical_con(positions, velocities, efforts_f)
    tau_ests, tau_exts = paraEstimator.testWithEstimatedParaIDyn(positions, velocities, ref_pam, estimate_pam)
    processed_data = combine_input_output(positions[1:], velocities[1:], tau_exts, tau_ests)
    
    for d in processed_data:
        csv_saveCreate(f"{prefix}/{filename}", d)

    return estimate_pam, ref_pam

def process_identificate_data(data, paraEstimator, traj_type='traj'):
    if traj_type == 'traj':
        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentList(data)
    elif traj_type == 'MC_test':
        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentListZeroVel(data)
    else:
        raise ValueError("Not define the type" + traj_type)
    efforts_f = efforts
    # 可以根据需要取消注释以滤除噪音
    # velocities = traj_filter(velocities)
    # efforts_f = traj_filter(efforts)
    
    estimate_pam, ref_pam = paraEstimator.timer_cb_regressor_physical_con(positions, velocities, efforts_f)
    # tau_ests, tau_exts = paraEstimator.testWithEstimatedParaIDyn(positions, velocities, estimate_pam, ref_pam)
    # processed_data = combine_input_output(positions[1:], velocities[1:], tau_exts, tau_ests)
    
    # for d in processed_data:
    #     csv_saveCreate(f"{prefix}/{filename}", d)

    return estimate_pam, ref_pam

def process_data_with_given_params(data, prefix, filename, paraEstimator, estimate_pam,ref_pam, traj_type='traj'):
    if traj_type == 'traj':
        positions, velocities, _ = paraEstimator.ExtractFromMeasurmentList(data)
    elif traj_type == 'MC_test':
        positions, velocities, _ = paraEstimator.ExtractFromMeasurmentListZeroVel(data)
    else:
        raise ValueError("Not define the type" + traj_type)
    # 可以根据需要取消注释以滤除噪音
    # velocities = traj_filter(velocities)
    # efforts_f = traj_filter(efforts)
    
    # estimate_pam, ref_pam = paraEstimator.timer_cb_regressor_physical_con(positions, velocities, efforts_f)
    tau_ests, tau_exts = paraEstimator.testWithEstimatedParaIDyn(positions, velocities, estimate_pam, ref_pam)
    processed_data = combine_input_output(positions[1:], velocities[1:], tau_exts, tau_ests)
    
    for d in processed_data:
        csv_saveCreate(f"{prefix}/{filename}", d)
    return True

def plot_params(ref_pam_list, estimate_pam_list):
    plt.figure(figsize=(10, 6))
    plt.scatter(range(len(ref_pam_list)), ref_pam_list, label='List 1', marker='o')
    plt.scatter(range(len(estimate_pam_list)), estimate_pam_list, label='List 2', marker='x')
    plt.title('Scatter Plot Comparison')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.legend()
    plt.show()



def view_channels(*measurements_list):
    plt.figure()
    DISPLAY = range(7)
    K = len(DISPLAY)
    for i, measurements in enumerate(measurements_list):
        pos = [[] for _ in DISPLAY]
        for mes in measurements:
            for i in range(len(DISPLAY)):
                pos[i].append(mes[DISPLAY[i]])
                # pos[1].append(mes[DISPLAY[1]])
                # pos[2].append(mes[DISPLAY[2]])
    
        for j in range(K):
            plt.subplot(K+1, 1, j+1)  # 三行一列，当前激活的是第一个图
            plt.plot(pos[j])  # '-r' 表示红色实线
            plt.grid(True)
        # plt.legend(loc='traj'+str(i))

        # plt.subplot(3, 1, 2)  # 三行一列，当前激活的是第一个图
        # plt.plot(pos[1])  # '-r' 表示红色实线
        # # plt.legend(loc='traj'+str(i))

        # plt.subplot(3, 1, 3)  # 三行一列，当前激活的是第一个图
        # plt.plot(pos[2])  # '-r' 表示红色实线
        # # plt.legend(loc='traj'+str(i))
    # 自动调整子图间距
    plt.tight_layout()
    # 显示图形
    plt.show()




def combine_input_output(pos, vel, eff, eff_predict):
    os = []
    for p,v,e,ep in zip(pos, vel, eff, eff_predict):
        err = np.asarray(ep)-np.asarray(e)
        os.append(p+v+e+err.tolist())

    return os


def csv_saveCreate(path, vector):
    directory = os.path.dirname(path)
    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(path,'a', newline='') as file:
        writer =csv.writer(file)
        writer.writerow(vector)
    return True







def main(args=None):
    rclpy.init(args=args)
    gravity_vec = [0.0, 0.0, -9.81] #[4.905, 0.0, -8.496]
    theta1 = 0.0
    theta2 = -0.5233

    Ff = 0.1
    sampling_rate = 100.0
    sampling_rate_inoptimization = 20.0

    file_name = "med7dock.urdf"
    paths = [
        os.path.join(
            get_package_share_directory("med7_dock_description")
        ),
        os.path.join(
            get_package_share_directory("lbr_description")
        )
    ]

    traj_instance = TrajGeneration(gravity_vector=gravity_vec)
    S,lbg,ubg,fc = traj_instance.get_optimization_problem(Ff = Ff,sampling_rate = sampling_rate_inoptimization, bias = [theta1, theta2, 0.0, 0.0, 0.0, 0.0, 0.0])


    instance = TrajectoryConductionSim(file_name, paths,is_traj_from_path=False,traj_data=None,gravity_vector=gravity_vec)
    paraEstimator = Estimator(gravity_vec=gravity_vec)

    prefix = "/home/thy/learningDynModel/meta/"

    for i in range(1,300,1):
        a,b = traj_instance.find_optimal_point_with_randomstart(S,lbg,ubg, Rank=5)
        print("a = {0} \n b = {1}".format(a,b))
        eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))
        conditional_num = np.sqrt(eigenvalues[0]/eigenvalues[-1])
        # print("conditional_num_best = ",conditional_num)

        values_list,keys = traj_instance.generateToList(a,b,Ff = Ff,sampling_rate=sampling_rate)



        instance.import_traj_fromlist(values_list)
        instance.set_friction_params()
        data = instance.run_sim_to_list()


        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentList(data)
        velocities=traj_filter(velocities)
        efforts_f=traj_filter(efforts)



        estimate_pam,ref_pam = paraEstimator.timer_cb_regressor_physical_con(positions, velocities, efforts_f)
        tau_exts, es =paraEstimator.testWithEstimatedParaCon(positions, velocities, efforts_f,estimate_pam)
        data = combine_input_output(positions, velocities, efforts, tau_exts)

        for d in data:
            csv_saveCreate(prefix+"{0}".format(i)+"/data_spt.csv", 
                    d
                    )


        # Test Part
        # a_n, b_n = traj_instance.trajectory_with_random()
        a_n,b_n = traj_instance.find_optimal_point_with_randomstart(S,lbg,ubg, Rank=5)
        values_list,keys = traj_instance.generateToList(a_n,b_n,Ff = Ff,sampling_rate=sampling_rate)

        instance.import_traj_fromlist(values_list)
        instance.set_friction_params()
        data = instance.run_sim_to_list()
        positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentList(data)
        velocities=traj_filter(velocities)
        efforts_f=traj_filter(efforts)
        tau_exts, es =paraEstimator.testWithEstimatedParaCon(positions, velocities, efforts_f,estimate_pam)

        data = combine_input_output(positions, velocities, efforts, tau_exts)

        for d in data:
            csv_saveCreate(prefix+"{0}".format(i)+"/data_qry.csv", 
                    d
                    )

    # nn_cp = er_state_compensator("/home/thy/learningDynModel/vae_simple_1_VAE_CNN3.pth")

    # outputs =[]
    # errs =[]
    # for p,v,e,ep in zip(positions, velocities, efforts_f, tau_exts):
    #     data_input = p+v+e
    #     output = nn_cp.predict(data_input)

    #     err = np.asarray(ep)-np.asarray(e)
    #     outputs.append(output)
    #     errs.append(err.tolist())

    

        # print("output = ",output)
        # print("err = ",err.tolist())









        # for d in data:
        #     csv_saveCreate(prefix+"{0}".format(i)+"/data.csv", 
        #             d
        #             )
    # paraEstimator.saveEstimatedPara(estimate_pam)
    # print("tau_exts = ",tau_exts)
    # compare_traj(outputs, errs)







    rclpy.shutdown()

if __name__ == "__main__":
    main()
