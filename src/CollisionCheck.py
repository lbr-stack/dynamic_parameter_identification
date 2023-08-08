#!/usr/bin/python3
import csv
import numpy as np
import optas
from optas.spatialmath import *

from ament_index_python import get_package_share_directory
import os
import math

from ParamsEstimation import getJointParametersfromURDF


def getConstraintsinJointSpace(robot,point_coord = [0.]*3,Nb=7, base_link="lbr_link_3", base_joint_name="lbr_joint_3", ee_link="lbr_link_ee"):
    q = cs.SX.sym('q', Nb, 1)

    pe = robot.get_global_link_position(ee_link, q)
    Re = robot.get_global_link_rotation(ee_link, q)

    pb = robot.get_global_link_position(base_link, q)
    Rb = robot.get_global_link_rotation(base_link, q)

    pp = pe + Re[:,0]*point_coord[0] + Re[:,1]*point_coord[1] + Re[:,2]*point_coord[2]

    robot_urdf = robot.urdf
    joint = robot_urdf.joint_map[base_joint_name]
    xyz, _ = robot.get_joint_origin(joint)

    bbox = cs.DM([0.2, 0.2, xyz[2]])
    p = Rb.T@(pp -pb)
    x= p[0]
    y= p[1]
    z= p[2]
    c = xyz[2]
    c = c/2
    a = 0.15
    b = 0.15
    EpVF = x*x/(a*a) + y*y/(b*b) + (z-c)*(z-c)/(c*c)-1

    # constraints = optas.

    p_fun = optas.Function('A_fun',[q],[EpVF])

    
    return p_fun 







def main():
    path = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            "med7dock.urdf.xacro",
        )
    
    robot = optas.RobotModel(
        xacro_filename=path,
        time_derivs=[1], #joint velocity
    )

    pfun =getConstraintsinJointSpace(robot,point_coord=[-0.25, 0.0, 0.1])
    pfun1 =getConstraintsinJointSpace(robot,point_coord=[-0.12, 0.0, 0.1])

    _,xyzs,__,___ = getJointParametersfromURDF(robot)

    q = np.array([0.0, 0.0, 0.0, 1.26, 0.0, -2.03, 0.0])
    p = pfun(q)
    p1 = pfun1(q)
    print("p = ",p)
    print("p1 = ",p1)

    # print("xyzs",xyzs)
if __name__ == "__main__":
    main()

