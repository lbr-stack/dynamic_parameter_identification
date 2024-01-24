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


sys.path.append('/home/thy/learningDynModel')
import torch
from er_app import er_state_compensator



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

    for i in range(300,305,1):
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
