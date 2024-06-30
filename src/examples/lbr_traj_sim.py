#!/usr/bin/python3
# import pybullet as p
# import pybullet_data
import csv
import os
from ament_index_python import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy
from rclpy import qos
from rclpy.node import Node
import trajsimulation 


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
    instance = trajsimulation.TrajectoryConductionSim(file_name, paths,traj_data = '/tmp/target_joint_states.csv')
    instance.set_friction_params()
    instance.run_sim()
    # instance.set_gravity_vector([4.905, 0.0, -8.496])
    # instance.run_sim()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

