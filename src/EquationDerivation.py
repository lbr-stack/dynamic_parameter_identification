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

import PyKDL as kdl



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

        # mi start from link 1(not base_link) to end-effector link
        m = cs.SX.sym('m', 1, Nb+1)
        # cmi start from link 1(not base_link) to end-effector link
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
            print("index i ={0}".format(i))
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
            _tau = ini.T @ iaxisis[i-1]

            taus.append(_tau)
            print(iaxisis[i-1])
            print(i-1)

        
        tau=cs.vertcat(*[taus[k] for k in range(len(taus)-2,-1,-1)])
        print(tau.size())


        # q = cs.SX.sym('q', Nb, 1)
        # qd = cs.SX.sym('qd', Nb, 1)
        # qdd = cs.SX.sym('qdd', Nb, 1)
        # m = cs.SX.sym('m', 1, Nb+1)
        # cm = cs.SX.sym('cm',3,Nb+1)
        # Icm = cs.SX.sym('Icm',3,3*Nb+3)
        self.dynamics_ = optas.Function('dynamics', [q,qd,qdd,m,cm,Icm], [tau])
        g_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,Icm)
        g1_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,np.zeros([3,3*Nb+3]))
        g2_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,np.ones([3,3*Nb+3]))
        
        self.gra = optas.Function('gravity', [q,m,cm], [g1_])
        

        q_np = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        g = self.gra(q_np,np.ones([1,Nb+1]),0.1*np.zeros([3,Nb+1]))
        
        print(g1_)
        print(g2_)

        print(g)

        # assert()

        # sub_dict = {qd: np.zeros([Nb,1]),qdd: np.zeros([Nb,1])}
        # g_tau = optas.substitute(tau, [qd,qdd],[cs.DM([0.0]*7),cs.DM([0.0]*7)])

        # q_np = np.zeros([Nb,1])
        # qd_np = np.zeros([Nb,1])
        # qdd_np = np.zeros([Nb,1])
        # m_np = np.ones([1,Nb+1])
        # cm_np = np.ones([3,Nb+1])
        # Icm_np = np.ones([3,3*Nb+3])
        # print("output = {0}".format(self.dynamics_(q_np,qd_np,qdd_np,m_np,cm_np,Icm_np)))
        # print("output2 = {0}".format(g_tau))

        

        # self.robot.urdf.get

        # pb.connect(
        # *[pb.DIRECT],
        # )
        # pb.resetSimulation()
        # gravz = -9.81
        # pb.setGravity(0, 0, gravz)

        # sampling_freq = 240
        # time_step = 1./float(sampling_freq)
        # pb.setTimeStep(time_step)
        # self.id = pb.loadURDF(
        #         path,
        #         basePosition=[0, 0, 0],
        #     )
        # print("output1 = {0}".format(pb.calculateInverseDynamics(self.id,q_np,qd_np,qdd_np)))

            




        








def main(args=None):
    rclpy.init(args=args)
    paraEstimator = Estimator()
    rclpy.spin(paraEstimator)
    rclpy.shutdown()



if __name__ == "__main__":
    main()