#!/usr/bin/python3
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy
from TrajGeneration import TrajGenerationUsrPath
from ament_index_python import get_package_share_directory
import numpy as np

def main(args=None):
    rclpy.init(args=args)

    # path_xarm = os.path.join(
    #         get_package_share_directory("gravity_compensation"),
    #         "urdf",
    #         "xarm",
    #         "xarm7base.urdf.xacro",
    #     )
    
    # path_xarm = os.path.join(
    #         get_package_share_directory("lbr_description"),
    #         "urdf",
    #         "med7",
    #         "med7.xacro",
    #     )

    path_xarm = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "urdf",
            "med",
            "med7dock.urdf.xacro",
        )
    
    # print("path_xarm = ",path_xarm)

    paraEstimator = TrajGenerationUsrPath(path=path_xarm, gravity_vector=[0,0,-9.81])
    Ff = 0.1
    sampling_rate = 100.0
    sampling_rate_inoptimization = 20.0

    theta1 = 0.0
    theta2 = -0.5233

    # a,b,fc = paraEstimator.generate_opt_traj_Link(Ff = Ff,sampling_rate = sampling_rate_inoptimization, bias = [0, 0, 0.0, 1.9, 0.0, 1.0, 0.0],q_min=[-6.2, -2.0, -6.2, -0.19, -6.2, -1.69, -6.2],q_max=[6.2, 2.0, 6.2, 3.94, 6.2, 3.14, 6.2])
    a,b,fc = paraEstimator.generate_opt_traj_Link(Ff = Ff,
                sampling_rate = sampling_rate_inoptimization, 
                bias = [0, 0, 0.0, 0.0, 0.0, 1.0, 0.0],
                q_min=[-6.2, -12.0, -16.2, -10.19, -16.2, -11.69, -16.2],
                q_max=[6.2, 12.0, 16.2, 13.94, 16.2, 13.14, 16.2]
                )
    # a,b,fc = paraEstimator.generate_opt_traj_Link(Ff = Ff,sampling_rate = sampling_rate_inoptimization)
    print("a = {0} \n b = {1}".format(a,b))

    ret = paraEstimator.generateToCsv(a,
                                    b,
                                    Ff = Ff,
                                    sampling_rate=sampling_rate, 
                                    path = "/tmp/target_joint_states.csv"
                                    )

    if ret:
        print("Done! Congratulations! self-collision avoidance")
        
        eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))

        print("fc = ",eigenvalues)
        print("a = {0} \n b = {1}".format(a,b))
        conditional_num = np.sqrt(eigenvalues[0]/eigenvalues[-1])
        print("conditional_num_best = ",conditional_num)

    rclpy.shutdown()


if __name__ == "__main__":
    main()