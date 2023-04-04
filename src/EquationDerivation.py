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

# import PyKDL as kdl
import urdf_parser_py.urdf as urdf
# import kdl_parser_py.urdf
# from urdf_parser_py.urdf import treeFromUrdfModel
# import urdf_parser_py.urdf as urdf


class Estimator(Node):
    def __init__(self, node_name = "para_estimatior", dt_ = 1.0) -> None:
        super().__init__(node_name=node_name)

        self.dt_ = dt_
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
        xyzs, rpys, axes = [], [], []
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
            axes.append(axis)
        print(" xyz, rpy, axis = {0}, {1} ,{2}".format(xyzs, rpys, axes))

        # 2. RNEA
        """
        input: q, qdot, qddot, model
        output: tau
        """
        Nb = self.robot.ndof
        self.Nb = Nb
        om0 = cs.DM([0.0,0.0,0.0])
        om0D = cs.DM([0.0,0.0,0.0])
        gravity_para = cs.DM([0.0, 0.0, -9.81])

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
                iRp = (rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])).T
                iaxisi = iRp @ axes[i]
                omi = iRp @ oms[i] + iaxisi* qd[i]
                omDi = iRp @ omDs[i] +  iRp @skew(oms[i]) @ (iaxisi*qd[i]) + iaxisi*qdd[i]
                # print("skew(omDs[i]) = {0}".format(skew(omDs[i]).size()))
                vDi = iRp @ vDs[i] + skew(omDi) @ xyzs[i]+ skew(omi) @ skew(omi)@ xyzs[i]
                
                fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@(skew(omi)@cm[:,i]))
                ni = Icm[:,i*3:i*3+3] @ omDi
                + skew(omi) @ Icm[:,i*3:i*3+3] @ omi
                #+ skew(cm[:,i]) @ fi
            else:
                print(joints_list_r1[i])
                iRp = rpy2r(rpys[i]) 
                iaxisi = iRp @ axes[i]
                omi = iRp @ oms[i]
                omDi = iRp @ omDs[i]
                # print("skew(omDs[i]) = {0}".format(skew(omDs[i]).size()))
                vDi = iRp @ vDs[i] + skew(omDi) @ xyzs[i]+ skew(omi) @ skew(omi)@ xyzs[i]
                # vDi = iRp @ (vDs[i] + skew(omDs[i]) @ xyzs[i]
                #             + skew(oms[i]) @ skew(oms[i])@ xyzs[i])
                
                fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@skew(omi)@cm[:,i])
                ni = Icm[:,i*3:i*3+3] @ omDi
                + skew(omi) @ Icm[:,i*3:i*3+3] @ omi
                #+ skew(cm[:,i]) @ fi

            

            oms.append(omi)
            omDs.append(omDi)
            vDs.append(vDi)
            fs.append(fi)
            ns.append(ni)
            iaxisis.append(iaxisi)

        # 8 joint; 9 links
        print("iaxisi = {0}".format(iaxisis))
        print("ns = {0}".format(len(ns)))
        print("xyzs = {0}".format(len(xyzs)))
        # joints_list_r2 = joints_list[1:]
        joints_list_r2 = joints_list_r

        pRi = rpy2r(rpys[-1])
        ifi = pRi @ fs[-1]#cs.DM([0.0,0.0,0.0])
        ini = #cs.DM([0.0,0.0,0.0])
        taus = []
        # pRi = I3()

        for i in range(len(joints_list_r2)-1,0,-1):
            # print(joints_list_r2[i])
            # print(len(fs))
            print("index = {0}".format(i))
            if(i < len(joints_list_r2)-1):
                pRi = rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])
            elif(i == len(joints_list_r2)-1):
                pRi = rpy2r(rpys[i])
            else:
                pRi = I3()
            
            # if(i<)
            
            # ini_new = ns[i+1] + pRi @ ini +skew(cm[:,i]) @ fs[i+1] +skew(xyzs[i]) @ pRi @ ifi
            # ifi = ifi_new
            # ini = ini_new
            #skew(cm[:,i]+xyzs[i]) 
            # ini = ns[i+1] + pRi @ ini +skew(cm[:,i]+xyzs[i]) @ fs[i+1] +pRi @skew(pRi.T @xyzs[i]) @ ifi
            ini = ns[i+1] + pRi @ ini +skew(cm[:,i]) @ fs[i+1] +skew(xyzs[i]) @ pRi @ifi
            ifi= pRi @ ifi + fs[i+1]
            pRi = rpy2r(rpys[i-1]) @ angvec2r(q[i-1], axes[i-1])
            # if(i != len(joints_list_r2)-1):
            _tau = ini.T @pRi.T @ axes[i-1]
            print("iaxisis[i] = {0}".format(pRi.T @ axes[i-1]))
            taus.append(_tau)
            # else:
            #     ini = ns[i] + skew(cm[:,i-1]+xyzs[i]) @ fs[i]
            #     # print("")
            #     ifi = fs[i]
        # for i in range(len(joints_list_r2)-1,-1,-1):
        #     print(joints_list_r2[i])
        #     # print(len(fs))
        #     print("index = {0}".format(i))
        #     if(i != len(joints_list_r2)-1):
        #         pRi = rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])
        #     else:
        #         pRi = rpy2r(rpys[i])
            
        #     ifi_new = pRi @ ifi + fs[i+1]
        #     ini_new = ns[i+1] + pRi @ ini +skew(cm[:,i]+xyzs[i]) @ fs[i+1] +pRi @skew(pRi.T @xyzs[i]) @ ifi
        #     # ini_new = ns[i+1] + pRi @ ini +skew(cm[:,i]) @ fs[i+1] +skew(xyzs[i]) @ pRi @ ifi
        #     ifi = ifi_new
        #     ini = ini_new

        #     pRi = rpy2r(rpys[i-1]) @ angvec2r(q[i-1], axes[i-1])
        #     # if(i != len(joints_list_r2)-1):
        #     _tau = ini.T @pRi.T @ axes[i]
        #     print("iaxisis[i] = {0}".format(axes[i-1]))
        #     taus.append(_tau)
            
            # print(iaxisis[i-1])
            # print(i-1)

        
        tau=cs.vertcat(*[taus[k] for k in range(len(taus)-1,-1,-1)])
        print(tau.size())
        urdf_string_ = xacro.process(path)
        robot = urdf.URDF.from_xml_string(urdf_string_)
        print([joint.name for joint in robot.joints if joint.origin is not None])
        print([joint.origin.xyz for joint in robot.joints if joint.origin is not None])




        print([link.name for link in robot.links if link.inertial is not None])
        print([link.inertial.origin.xyz for link in robot.links if link.inertial is not None])
        print([link.inertial.mass for link in robot.links if link.inertial is not None])
        masses = [link.inertial.mass for link in robot.links if link.inertial is not None]#+[1.0]
        self.masses_np = np.array(masses[1:])
        print("masses = {0}".format(self.masses_np))

        massesCenter = [link.inertial.origin.xyz for link in robot.links if link.inertial is not None]#+[[0.0,0.0,0.0]]
        self.massesCenter_np = np.array(massesCenter[1:]).T
        print("massesCenter = {0}".format(self.massesCenter_np))


        # q = cs.SX.sym('q', Nb, 1)
        # qd = cs.SX.sym('qd', Nb, 1)
        # qdd = cs.SX.sym('qdd', Nb, 1)
        # m = cs.SX.sym('m', 1, Nb+1)
        # cm = cs.SX.sym('cm',3,Nb+1)
        # Icm = cs.SX.sym('Icm',3,3*Nb+3)
        self.dynamics_ = optas.Function('dynamics', [q,qd,qdd,m,cm,Icm], [tau])
        g_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,Icm)
        g1_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,np.zeros([3,3*Nb+3]))
        # g2_ =  self.dynamics_(q,np.zeros([Nb,1]),np.zeros([Nb,1]),m,cm,np.ones([3,3*Nb+3]))
        
        self.gra = optas.Function('gravity', [q,m,cm], [g1_])

        self.lbr_command_timer_ = self.create_timer(self.dt_, self.timer_cb_)

        pb.connect(*[pb.DIRECT])
        pb.resetSimulation()
        path2 = os.path.join(
            get_package_share_directory("lbr_description"),
            "urdf",
            self.model_,
        )
        pb.setAdditionalSearchPath(path2)

        gravz = -9.81
        pb.setGravity(0, 0, gravz)

        sampling_freq = 240
        time_step = 1./float(sampling_freq)
        pb.setTimeStep(time_step)
        pb.resetDebugVisualizerCamera(
            cameraDistance=0.2,
            cameraYaw=-180,
            cameraPitch=30,
            cameraTargetPosition=np.array([0.35, -0.2, 0.2]),
        )
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

        self.id = pb.loadURDF(
            'med7.urdf',
            basePosition=[0, 0, 0],
        )
        self.iter = 0.0


        
    def timer_cb_(self) -> None:
        q_np = np.array([1.0, self.iter+1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        qd_np = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.iter += 3.1415926535
        for i in range(self.Nb):
            # print("ccc = {0}".format(i))
            pb.resetJointState(self.id, i, q_np[i], qd_np[i])

        tau_ext = np.array(pb.calculateInverseDynamics(self.id, q_np.tolist(), qd_np.tolist(), np.zeros(self.Nb).tolist()))
        g = self.gra(q_np,self.masses_np,self.massesCenter_np)
        
        print("tau_ext = {0}\n tau_g = {1}".format(tau_ext,g))
        print("\n error = {0}\n ".format(tau_ext-g))
        # urdf_string_ = xacro.process(path)

        # # Load the URDF file and create a PyKDL.Tree object
        # robot = urdf.URDF.from_xml_string(urdf_string_)
        # # tree = treeFromUrdfModel(robot)

        # # Define the base and end links of the chain
        
        # # end_link = "lbr_link_ee"
        # # print(robot.links[9].name)
        # # Get the chain between the base and end links
        # chain = robot.get_chain(robot.links[1].name, robot.links[9].name)
        # # tree = urdf.URDF.treeFromUrdfModel(robot)
        # # chain = tree.getChain(robot.links[1].name, robot.links[9].name)

        # gravity = kdl.Vector(0, 0, -9.81)
        # dyn = kdl.ChainDynParam(chain, gravity)

        # gravity_torque = kdl.JntArray(chain.getNrOfJoints())
        # dyn.JntToGravity(q_np, gravity_torque)


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