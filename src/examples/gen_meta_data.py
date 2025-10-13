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
from trajsimulation import TrajectoryConductionSim
from regression import Estimator,traj_filter,compare_traj
from utility_math import csv_save
from MetaGen import combine_input_output, csv_saveCreate

import rospkg
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
from MetaGen import generate_excitation_sat_path,process_regression_data,plot_params,process_data_with_given_params,view_channels,process_identificate_data


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


import re
import pandas as pd
import random

def generate_friction_coefficients():
    """
    Generates a 14-dimensional NumPy array with 7 Coulomb friction coefficients and 7 viscous friction coefficients.
    The first four coefficients are higher to simulate the characteristics of larger joints.

    Returns:
        np.ndarray: A 14-dimensional array containing 7 Coulomb and 7 viscous friction coefficients.
    """
    coulomb_friction = []
    viscous_friction = []

    for joint in range(7):
        if joint < 4:
            # Higher friction for the first four joints
            coulomb_friction_range = (2.0, 3.0)  # Nm, higher range
            viscous_friction_range = (2.0, 3.335)  # Nm·s/rad, higher range
        else:
            # Lower friction for the remaining joints
            coulomb_friction_range = (0.1, 1.0)  # Nm, lower range
            viscous_friction_range = (0.001, 1.0)  # Nm·s/rad, lower range

        # Generate random values within the specified ranges
        coulomb_friction.append(random.uniform(*coulomb_friction_range))
        viscous_friction.append(random.uniform(*viscous_friction_range))

    # Combine both lists into a 14-dimensional NumPy array
    friction_coefficients = np.array(coulomb_friction + viscous_friction)
    
    return friction_coefficients

def generate_rotated_vectors(num_rotations):
    # Ensure the number of rotations is a positive integer
    if num_rotations <= 0:
        raise ValueError("Number of rotations must be a positive integer.")

    # Initial vector
    vector = np.array([0.0, 0.0, -9.81])

    # Calculate the rotation angles (in degrees)
    angles = -np.linspace(0, 90, num_rotations)

    # List to store the rotated vectors
    rotated_vectors = []

    # Rotation around the Y-axis
    for angle in angles:
        rad = np.deg2rad(angle)  # Convert angle to radians
        rotation_matrix = np.array([
            [np.cos(rad), 0, np.sin(rad)],
            [0, 1, 0],
            [-np.sin(rad), 0, np.cos(rad)]
        ])
        rotated_vector = np.dot(rotation_matrix, vector)
        rotated_vectors.append(rotated_vector.tolist())  # Store the full 3D vector

    return rotated_vectors

def process_files(root_dir, target_idc):
    pattern = r"traj_(\d+\.\d+)_(\d+)\.csv"  # Regex to match conditional_num and idc
    values_list = None  # Initialize values_list to None
    files_found = False  # Flag to check if any files were found

    for subdir, _, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".csv"):
                match = re.search(pattern, file)
                if match:
                    conditional_num = match.group(1)
                    idc = match.group(2)
                    
                    # Ensure the extracted idc matches the input idc
                    if int(idc) != target_idc:
                        continue  # Skip files that do not match the target idc
                    
                    # Extract ff from the directory path
                    # parts = subdir.split('/')
                    # try:
                    #     ff = float(parts[-1][1:])  # Assuming directory is like 't10.0'
                    # except ValueError:
                    #     continue  # Skip directories that do not match the expected pattern
                    
                    # print("conditional_num= ",file)
                    # Construct the full file path
                    file_path = os.path.join(subdir, file)
                    
                    # Read the CSV file into a DataFrame
                    values_list = pd.read_csv(file_path).values.tolist()
                    
                    # Set the flag to True indicating a file was found and processed
                    files_found = True
                    print(f"Processing file: {file_path}")
                    return values_list  # Return the first matching file's data
    
    # If no matching files were found, raise an alert
    if not files_found:
        raise FileNotFoundError(f"No files with idc {target_idc} found in directory {root_dir}")

            




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
    N_dyna = 10 # Number of Dynamics

    rotated_vectors = generate_rotated_vectors(N_dyna)
    print(rotated_vectors[1])
    # raise ValueError("11")

    N_traj = 20 # Number of Trajs
    N_fric = 1 # Number of frictions
    prefix = "/home/thy/excitation_trajs"

    prefix_save = "/home/thy/MetaDynLearnSave/"
    pfdir= 0
    for i in range(N_traj):
        #regression simplication
        """1.这里可以换成读取的"""
        print(prefix+"/t{0}/".format((i+1)*10.0))
        values_list_spt = process_files(prefix+"/t{0}/".format((i+1)*10.0), 0)
        values_list_qry = process_files(prefix+"/t{0}/".format((i+1)*10.0), 1)



        for k in range(0, N_dyna):
            """2. different """
            instance = TrajectoryConductionSim(file_name, 
                                            paths,is_traj_from_path=False,
                                            traj_data=None,
                                            gravity_vector=rotated_vectors[k],
                                            use_gui=False)
            paraEstimator = Estimator(gravity_vec=rotated_vectors[k])
            print("rotated_vectors[k] = ",rotated_vectors[k])

            # raise ValueError("111")
            for j in range(N_fric):
                """2. change parameters of frictions"""
                ref_pam = np.array(paraEstimator.get_gt_params_simO())
                print("ref_pam", ref_pam.shape)

                # raise ValueError("111")
                fric_pam = generate_friction_coefficients()
                ref_pam[-14:] =fric_pam
                
                instance.setup_params_sim(ref_pam)

                instance.import_traj_fromlist(values_list_spt)
                data_spt = instance.run_sim_to_list()

                instance.import_traj_fromlist(values_list_qry)
                data_qry = instance.run_sim_to_list()

                estimate_pam, _ = process_identificate_data(data_spt, paraEstimator)

                pfdir = pfdir+1
                """TODO :"""
                process_data_with_given_params(data_spt, prefix_save+str(pfdir), "/data_spt.csv", 
                                            paraEstimator,
                                            estimate_pam, 
                                            ref_pam)
                process_data_with_given_params(data_qry, prefix_save+str(pfdir), "/data_qry.csv", 
                                            paraEstimator, 
                                                estimate_pam,
                                                ref_pam)
 
    
    rclpy.shutdown()





# # Example usage:
# if __name__ == "__main__":
#     num_rotations = 5
#     rotated_vectors = generate_rotated_vectors(num_rotations)
#     print(rotated_vectors)

if __name__ == "__main__":
    main()

