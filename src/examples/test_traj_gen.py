#!/usr/bin/python3

import os

from ament_index_python import get_package_share_directory
import rclpy
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajsimulation import TrajectoryConductionSim

from MetaGen import generate_excitation_sat_path,process_regression_data,plot_params,process_data_with_given_params,view_channels
from TrajGeneration import save_to_csv, load_from_csv


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

    # Init a simulation environment
    instance = TrajectoryConductionSim(file_name, paths,is_traj_from_path=False,traj_data=None,gravity_vector=gravity_vec)


    #regression simplication
    """111"""
    """ Note: Ff decides the cycle time of the Traj.
            Make Ff smaller, T will be longer
        """
    
    idc = 1
    
    ff_list = [1 / i for i in range(10, 201,10)]
    for ff in ff_list:
        values_list,conditional_num = generate_excitation_sat_path(path_arm, 
                                                                   gravity_vec,
                                                                   inopt_rate=20*ff*10,
                                                                   Ff=ff,
                                                                   cond_th=1e10)
        save_to_csv(values_list, "/home/thy/excitation_trajs/t{1}/traj_{0}_{2}.csv".format(conditional_num,1/ff,idc))

    

    

if __name__ == "__main__":
    main()
