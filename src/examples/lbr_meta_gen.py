#!/usr/bin/python3
import pybullet as p
import pybullet_data
import time
import csv
import os
import xacro
from ament_index_python import get_package_share_directory,get_package_prefix
import rclpy
from rclpy import qos
from rclpy.node import Node
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from TrajGeneration import TrajGenerationUsrPath
from trajsimulation import TrajectoryConductionSim
from regression import Estimator,traj_filter,compare_traj
from utility_math import csv_save
from MetaGen import combine_input_output, csv_saveCreate

import rospkg
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
from MetaGen import generate_excitation_sat_path,process_regression_data,plot_params,process_data_with_given_params,view_channels,process_identificate_data
import random
module_path = os.path.join(
            get_package_prefix("gravity_compensation"),
            "lib",
            "gravity_compensation"
        )
sys.path.append(module_path)
from trajsimulation import replace_package_paths_in_xacro

module_path1 = os.path.join(
            get_package_prefix("path_following_pipeline"),
            "lib",
            "path_following_pipeline"
        )
sys.path.append(module_path1)
import math

from pb_offline_sim import move_robot_to_start,get_non_fixed_joints,move_robot_along_path
from robot_path_planning import RobotPathGenerator,generate_circle_path_with_orientation,generate_target_path,add_random_offset


def main(args=None):

    # define parameters
    rclpy.init(args=args)
    gravity_vec = [0.0, 0.0, -9.81] #[4.905, 0.0, -8.496]

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

    path_arm = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "urdf",
            "med",
            "med7dock.urdf.xacro",
        )


    # Test

    # Generate a excitation trajectory



    # Init a simulation environment
    instance = TrajectoryConductionSim(file_name, paths,is_traj_from_path=False,traj_data=None,gravity_vector=gravity_vec,use_gui=False)
    paraEstimator = Estimator(gravity_vec=gravity_vec)

    N_exc = 10
    N_traj = 50
    offset = 119
    prefix = "/home/thy/learningDynModel/meta/"
    for i in range(N_exc):
        #regression simplication
        """111"""
        values_list,_ = generate_excitation_sat_path(path_arm, gravity_vec)
        instance.import_traj_fromlist(values_list)
        instance.set_friction_params()

        _org_excitation = instance.run_sim_to_list()
        estimate_pam, ref_pam = process_identificate_data(_org_excitation, paraEstimator)
        # Example usage
        for j in range(N_traj):
            # Identify the parameters
            """222"""

            center = np.array([0, random.uniform(0.3, 0.6), random.uniform(0.3, 0.6)])
            radius = random.uniform(0.1, 0.3)
            normal = np.array([random.uniform(0, 2*math.pi), random.uniform(0, 2*math.pi), random.uniform(0, 2*math.pi)])
            normal = normal/np.linalg.norm(normal)

            circle_path = generate_circle_path_with_orientation(center, radius, normal,num_points=1000)
            if circle_path:
                pass
            else:
                print("circle_path = ",circle_path)
                raise ValueError("111")

            
            data_ = generate_target_path(circle_path)
            amended_tj=add_random_offset(data_)


            pfdir = j+N_traj*i + offset

            process_data_with_given_params(data_, prefix+str(pfdir), "/data_spt.csv", paraEstimator,estimate_pam, ref_pam)
            process_data_with_given_params(amended_tj, prefix+str(pfdir), "data_qry.csv", paraEstimator, 
                                                                    estimate_pam,ref_pam)

    # print("target_js_list = ",target_js_list)
    # raise ValueError("111"+c)
    
    # """333"""
    # ref_pam_list = ref_pam.flatten().tolist()
    # estimate_pam_list = estimate_pam.full().flatten().tolist()
    # plot_params(ref_pam_list, estimate_pam_list)

    # positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentListZeroVel(data_)
    # tau_ests, tau_exts =paraEstimator.testWithEstimatedParaIDyn(positions, velocities, estimate_pam,ref_pam)
    # view_channels(tau_ests,tau_exts)

    
    
    rclpy.shutdown()



if __name__ == "__main__":
    main()
