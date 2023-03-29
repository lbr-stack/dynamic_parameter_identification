#!/usr/bin/python3
import optas
import sys
import numpy as np
import pybullet as pb
import matplotlib.pyplot as plt
from time import sleep, time, perf_counter, time_ns
from scipy.spatial.transform import Rotation as Rot
from optas.spatialmath import *
import os

import rclpy
import xacro
from ament_index_python import get_package_share_directory
from rclpy import qos
from rclpy.node import Node

import pathlib



class Estimator(Node):
    def __init__(self, node_name = "para_estimatior") -> None:
        super().__init__(node_name=node_name)

        self.declare_parameter("model", "med7")
        self.model_ = str(self.get_parameter("model").value)
        path = os.path.join(
            get_package_share_directory("lbr_description"),
            "urdf",
            self.model_,
            f"{self.model_}.urdf.xacro",
        )

        # self.urdf_string_ = xacro.process(path)

        # 1. Get the kinematic parameters of every joints
        self.robot = optas.RobotModel(
            xacro_filename=path,
            time_derivs=[1],  # i.e. joint velocity
        )

        root = self.robot.urdf.get_root()

        ee_link = "lbr_link_ee"
        xyzs, rpys, axiss = [], [], []
        joints_list = self.robot.urdf.get_chain(root, ee_link, links=False)
        print("joints_list = {0}"
              .format(joints_list)
              )
        joints_list_r = joints_list[1:]
        for joint_name in joints_list_r:
            print(joint_name)
            joint = self.robot.urdf.joint_map[joint_name]
            xyz, rpy = self.robot.get_joint_origin(joint)
            axis = self.robot.get_joint_axis(joint)

            # record the kinematic parameters
            xyzs.append(xyz)
            rpys.append(rpy)
            axiss.append(axis)
            print(" xyz, rpy, axis = {0}, {1} ,{2}".format(xyz, rpy, axis))

        # 2. RNEA
        """
        input: q, qdot, qddot, model
        output: tau
        """
        Nb = self.robot.ndof
        om0 = cs.DM([0.0,0.0,0.0])
        om0D = cs.DM([0.0,0.0,0.0])
        gravity_para = cs.DM([0.0, 0.0, 9.8])

        q = cs.SX.sym('q', Nb, 1)
        qd = cs.SX.sym('qd', Nb, 1)
        qdd = cs.SX.sym('qdd', Nb, 1)
        m = cs.SX.sym('m', 1, Nb+1)
        cm = cs.SX.sym('cm',3,Nb+1)
        Icm = cs.SX.sym('Icm',3,3*Nb+3)
        fs = [cs.DM([0.0,0.0,0.0])]
        ns = [cs.DM([0.0,0.0,0.0])]

        oms = [om0]
        omDs = [om0D]
        vDs = [-gravity_para]
        iaxisis = []
        
        # 2.1 forward part of RNEA
        # print("axiss = {}")
        RL = I3()

        # joint 0 -> q1
        # qd[i] -> the joint speed at next link (base_link -> link_1)
        # p: last link
        # i: next link
        # oms[i] omega velocity angle at p th (not i but p)
        # xyzs[i]: i-1 to i link
        joints_list_r1 = joints_list_r
        for i in range(len(joints_list_r1)):
            if(i!=len(joints_list_r1)-1):
                print(joints_list_r1[i])
                iRp = (rpy2r(rpys[i]) @ angvec2r(q[i], axiss[i])).T
                iaxisi = iRp @ axiss[i]
                omi = iRp @ oms[i] + iaxisi* qd[i]
                omDi = iRp @ omDs[i] + skew(iRp @ oms[i]) @ (iaxisi*qd[i]) + iaxisi*qdd[i]
                # print("skew(omDs[i]) = {0}".format(skew(omDs[i]).size()))
                vDi = iRp @ (vDs[i] + skew(omDs[i]) @ xyzs[i]
                            + skew(oms[i]) @ skew(oms[i])@ xyzs[i])
                
                fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@skew(omi)@cm[:,i])
                ni = Icm[:,i*3:i*3+3] @ omDi
                + skew(omi) @ Icm[:,i*3:i*3+3] @ omi
                + skew(cm[:,i]) @ fi
            else:
                print(joints_list_r1[i])
                iRp = rpy2r(rpys[i]) 
                iaxisi = iRp @ axiss[i]
                omi = iRp @ oms[i]
                omDi = iRp @ omDs[i]
                # print("skew(omDs[i]) = {0}".format(skew(omDs[i]).size()))
                vDi = iRp @ (vDs[i] + skew(omDs[i]) @ xyzs[i]
                            + skew(oms[i]) @ skew(oms[i])@ xyzs[i])
                
                fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@skew(omi)@cm[:,i])
                ni = Icm[:,i*3:i*3+3] @ omDi
                + skew(omi) @ Icm[:,i*3:i*3+3] @ omi
                + skew(cm[:,i]) @ fi

            

            oms.append(omi)
            omDs.append(omDi)
            vDs.append(vDi)
            fs.append(fi)
            ns.append(ni)
            iaxisis.append(iaxisi)

        # 8 joint; 9 links
        # joints_list_r2 = joints_list[1:]
        joints_list_r2 = joints_list_r
        ifi = cs.DM([0.0,0.0,0.0])
        ini = cs.DM([0.0,0.0,0.0])
        taus = []
        for i in range(len(joints_list_r2)-1,-1,-1):
            print(joints_list_r2[i])
            print(i)
            # print(len(fs))
            if(i != len(joints_list_r2)-1):
                pRi = rpy2r(rpys[i]) @ angvec2r(q[i], axiss[i])
            else:
                pRi = rpy2r(rpys[i])
            ifi_new = pRi @ ifi + fs[i+1]
            ini_new = ns[i+1] + pRi @ ini +skew(cm[:,i]) @ fs[i+1] +skew(xyzs[i]) @ pRi @ ifi

            ifi = ifi_new
            ini = ini_new

            # if(i != len(joints_list_r2)-1):
            tau = ini.T @ iaxisis[i-1]

            taus.append(tau)
            print(tau)
            


            # print(joints_list_r2[i])
            # print(i)
            # print(len(joints_list_r2)-1)

        


        






        # for i in range(len(joints_list_r)):
        #     # print(joints_list_r[i])
        #     pRi=rpy2r(axiss[i]*q[i]).T
        #     iRp = rpy2r(axiss[i]*q[i]) @ RL
        #     omi = iRp @ oms[i] + axiss[i]* qd[i]
        #     omDi = iRp @ omDs[i] + skew(iRp @ oms[i]) @ (axiss[i]*qd[i]) + axiss[i]*qdd[i]
        #     # print("skew(omDs[i]) = {0}".format(skew(omDs[i]).size()))
        #     vDi = iRp @ (vDs[i] + skew(omDs[i]) @ xyzs[i])
        #     vDi = iRp @ (vDs[i] + skew(omDs[i]) @ xyzs[i]
        #                  + skew(oms[i]) @ skew(oms[i])@ xyzs[i])
            
        #     fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@skew(omi)@cm[:,i])
        #     ni = Icm[:,i*3:i*3+3] @ omDi
        #     + skew(omi) @ Icm[:,i*3:i*3+3] @ omi
        #     + skew(cm[:,i]) @ fi
            
        #     RL = iRp
            
        #     print(ni)

        #     oms.append(omi)
        #     omDs.append(omDi)
        #     vDs.append(vDi)
        #     fs.append(fi)
        #     ns.append(ni)



        








def main(args=None):
    rclpy.init(args=args)
    paraEstimator = Estimator()
    rclpy.spin(paraEstimator)
    rclpy.shutdown()



if __name__ == "__main__":
    main()