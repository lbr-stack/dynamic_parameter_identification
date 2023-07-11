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
import csv

import pathlib

import urdf_parser_py.urdf as urdf
import math
import copy

# Order = [4,1,2,3,5,6,7]
Order = [0,1,2,3,4,5,6]

# def GetOrder(file_path):

#     return order

def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

def fhan(x1, x2, u, r, h):
    d = r * h
    d0 = d* h
    y = x1 - u+h*x2
    a0 = math.sqrt(d*d + 8*r*abs(y))

    if abs(y) <= d0:
        a = x2 + y/h
    else:
        a = x2+0.5*(a0-d)*sign(y)

    if abs(a)<=d:
        return -r*a/d
    else:
        return -r*sign(a)
    
class TD_2order:
    def __init__(self, T=0.01, r=10.0, h=0.1):
        self.x1 = None
        self.y1 = None
        self.T = T
        self.r = r
        self.h = h

    def __call__(self, u):
        if self.x1 is None or self.x2 is None:
            self.x1 = 0
            self.x2 = 0

        x1k = self.x1
        x2k = self.x2
        self.x1 = x1k + self.T* x2k
        self.x2 = x2k + self.T* fhan(x1k, x2k, u, self.r, self.h)

        return self.x1, self.x2


def RNEA_function(Nb,Nk,rpys,xyzs,axes):
    # 2. RNEA
    Nf = Nb+Nk
    """
    input: q, qdot, qddot, model
    output: tau
    """
    
    om0 = cs.DM([0.0,0.0,0.0])
    om0D = cs.DM([0.0,0.0,0.0])
    gravity_para = cs.DM([0.0, 0.0, -9.81])
    # gravity_para = cs.DM([4.905, 0.0, -8.496])

    """
    The definination of joint position from joint0 to joint(Nb-1)
    """

    q = cs.SX.sym('q', Nb, 1)
    qd = cs.SX.sym('qd', Nb, 1)
    qdd = cs.SX.sym('qdd', Nb, 1)

    """
    The definination of mass for link1 to linkee
    The definination of center of Mass for link1 to linkee
    The definination of Inertial tensor for link1 to linkee
    """
    m = cs.SX.sym('m', 1, Nb+1)
    cm = cs.SX.sym('cm',3,Nb+1)
    Icm = cs.SX.sym('Icm',3,3*Nb+3)

    """
    external force given by link0
    The list will be appended via RNEA
    """
    fs = [cs.DM([0.0,0.0,0.0])]
    ns = [cs.DM([0.0,0.0,0.0])]


    """
    $$ Forward part of RNEA $$
    oms,omDs,vDs given by link0. The list will be appended via RNEA.
    Notes: the gravity_para represents a base acceration to subsitute the gravity.

    """
    oms = [om0]
    omDs = [om0D]
    vDs = [-gravity_para]
    
    
    # 2.1 forward part of RNEA
    """
    link0->1, link1->2 ...., link7->end-effector (for example)
    joint0,   joint1 ....
    
    """
    
    for i in range(Nf):
        print("link p({0}) to i{1}".format(i,i+1))
        if(i!=Nf-1):
            # print(joints_list_r1)
            iRp = (rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])).T
            iaxisi = iRp @ axes[i]
            omi = iRp @ oms[i] + iaxisi* qd[i]
            omDi = iRp @ omDs[i] +  skew(iRp @oms[i]) @ (iaxisi*qd[i]) + iaxisi*qdd[i]
        else:
            iRp = rpy2r(rpys[i]).T
            omi = iRp @ oms[i]
            omDi = iRp @ omDs[i]

        vDi = iRp @ (vDs[i] 
                        + skew(omDs[i]) @ xyzs[i]
                    + skew(oms[i]) @ (skew(oms[i])@ xyzs[i]))
        
        fi = m[i] * (vDi + skew(omDi)@ cm[:,i]+ skew(omi)@(skew(omi)@cm[:,i]))
        ni = Icm[:,i*3:i*3+3] @ omDi + skew(omi) @ Icm[:,i*3:i*3+3] @ omi #+ skew(cm[:,i]) @ fi
        

        oms.append(omi)
        omDs.append(omDi)
        vDs.append(vDi)
        fs.append(fi)
        ns.append(ni)


    """
    $$ Backward part of RNEA $$
    """

    # pRi = rpy2r(rpys[-1])
    ifi = fs[-1]#cs.DM([0.0,0.0,0.0])
    ini = ns[-1] + skew(cm[:,-1]) @ fs[-1]#cs.DM([0.0,0.0,0.0])
    # ifi = cs.DM([0.0,0.0,0.0])
    # ini = cs.DM([0.0,0.0,0.0])
    taus = []

    # print("Backward: fs[i+1] {0}".format(len(fs)))
    for i in range(Nf-1,0,-1):

        print("Backward: link i({0}) to p{1}".format(i+1,i))
        # print("Backward: fs[i+1]".format(fs[i+1]))
        if(i < Nf-1):
            pRi = rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])
        elif(i == Nf-1):
            pRi = rpy2r(rpys[i])
        else:
            pRi = rpy2r(rpys[i])
        

        ini = ns[i] + pRi @ ini +skew(cm[:,i-1]) @ fs[i] +skew(xyzs[i]) @ pRi @ifi
        ifi= pRi @ ifi + fs[i]
        pRi = rpy2r(rpys[i-1]) @ angvec2r(q[i-1], axes[i-1])
        _tau = ini.T @pRi.T @ axes[i-1]
        taus.append(_tau)


        
    tau_=cs.vertcat(*[taus[k] for k in range(len(taus)-1,-1,-1)])
    dynamics_ = optas.Function('dynamics', [q,qd,qdd,m,cm,Icm], [tau_])
    return dynamics_



def DynamicLinearlization(dynamics_,Nb):
    """
    The definination of joint position from joint0 to joint(Nb-1)
    """

    q = cs.SX.sym('q', Nb, 1)
    qd = cs.SX.sym('qd', Nb, 1)
    qdd = cs.SX.sym('qdd', Nb, 1)

    """
    The definination of mass for link1 to linkee
    The definination of center of Mass for link1 to linkee
    The definination of Inertial tensor for link1 to linkee
    """
    m = cs.SX.sym('m', 1, Nb+1)
    cm = cs.SX.sym('cm',3,Nb+1)
    Icm = cs.SX.sym('Icm',3,3*Nb+3)

    
    Y = []
        
    for i in range(Nb):
        # for every link
        # Y_line = []
        Y_line = []
        # PI_a = []
        for j in range(m.shape[1]):
            # for every parameters
            ## 1. get mass
            m_indu = np.zeros([m.shape[1],m.shape[0]])
            cm_indu = np.zeros([3,Nb+1])#np.zeros([cm.shape[1],cm.shape[0]])
            Icm_indu = np.zeros([3,3*Nb+3])#np.zeros([Icm.shape[1],Icm.shape[0]])
            # print(*m.shape)
            m_indu[j] = 1.0
            # print(m_indu)

            output = dynamics_(q,qd,qdd,m_indu,cm_indu,Icm_indu)[i]
            Y_line.append(output)


            ## 2. get cmx
            output1 = dynamics_(q,qd,qdd,m_indu,cm,Icm_indu)[i]-output
            for k in range(3):
                output_cm = optas.jacobian(output1,cm[k,j])
                output_cm1 = optas.substitute(output_cm,cm,cm_indu)
                Y_line.append(output_cm1)

            ## 3.get Icm
            output2 = dynamics_(q,qd,qdd,m_indu,cm_indu,Icm)[i]-output
            for k in range(3):
                for l in range(k,3,1):
                    output_Icm = optas.jacobian(output2,Icm[k,l+3*j])
                    Y_line.append(output_Icm)

            # sx_lst = optas.horzcat(*Y_seg)
            # Y_line
        sx_lst = optas.horzcat(*Y_line)
        Y.append(sx_lst)
        # print("Y_line shape = {0}, {1}".format(Y_line[0].shape[0],Y_line[0].shape[1]))
        # print("sx_lst shape = {0}, {1}".format(sx_lst.shape[0],sx_lst.shape[1]))

    Y_mat = optas.vertcat(*Y)
    # print(Y_mat)
    print("Y_mat shape = {0}, {1}".format(Y_mat.shape[0],Y_mat.shape[1]))
    Ymat = optas.Function('Dynamic_Ymat',[q,qd,qdd],[Y_mat])

    PI_a = []
    for j in range(m.shape[1]):
        # for every parameters
        pi_temp = [m[j],
                    m[j]*cm[0,j],
                    m[j]*cm[1,j],
                    m[j]*cm[2,j],
                    Icm[0,0+3*j] + m[j]*(cm[1,j]*cm[1,j]+cm[2,j]*cm[2,j]),  # XXi
                    Icm[0,1+3*j] - m[j]*(cm[0,j]*cm[1,j]),  # XYi
                    Icm[0,2+3*j] - m[j]*(cm[0,j]*cm[2,j]),  # XZi
                    Icm[1,1+3*j] + m[j]*(cm[0,j]*cm[0,j]+cm[2,j]*cm[2,j]),  # YYi
                    Icm[1,2+3*j] - m[j]*(cm[1,j]*cm[2,j]),  # YZi
                    Icm[2,2+3*j] + m[j]*(cm[0,j]*cm[0,j]+cm[1,j]*cm[1,j])] # ZZi
        PI_a.append(optas.vertcat(*pi_temp))

    PI_vecter = optas.vertcat(*PI_a)
    print("PI_vecter shape = {0}, {1}".format(PI_vecter.shape[0],PI_vecter.shape[1]))
    PIvector = optas.Function('Dynamic_PIvector',[m,cm,Icm],[PI_vecter])

    return Ymat, PIvector

def find_eigen_value(dof, parm_num, regressor_func,shape):
    '''
    Find dynamic parameter dependencies (i.e., regressor column dependencies).
    '''

    samples = 100
    round = 10

    pi = np.pi

    # Z = np.zeros((dof * samples, parm_num))
    A_mat = np.zeros(( shape,shape ))

    for i in range(samples):
        a = np.random.random([parm_num,dof])*2.0-1.0
        b = np.random.random([parm_num,dof])*2.0-1.0

        A_mat = A_mat+ regressor_func(a,b)

    print("U V finished")
    U, s, V = np.linalg.svd(A_mat)
        # Z[i * dof: i * dof + dof, :] = np.matrix(
        #     regressor_func(q, dq, ddq)).reshape(dof, parm_num)

    # R1_diag = np.linalg.qr(Z, mode='r').diagonal().round(round)
    

    return U,V

def getJointParametersfromURDF(robot, ee_link="lbr_link_ee"):
    robot_urdf = robot.urdf
    root = robot_urdf.get_root()
    # ee_link = "lbr_link_ee"
    xyzs, rpys, axes = [], [], []


    joints_list = robot_urdf.get_chain(root, ee_link, links=False)
    # print("joints_list = {0}"
    #         .format(joints_list)
    #         )
    # assumption: The first joint is fixed. The information in this joint is not recorded
    """
    xyzs starts from joint 0 to joint ee
    rpys starts from joint 0 to joint ee
    axes starts from joint 0 to joint ee
    """
    joints_list_r = joints_list[1:]
    for joint_name in joints_list_r:
        print(joint_name)
        joint = robot_urdf.joint_map[joint_name]
        xyz, rpy = robot.get_joint_origin(joint)
        axis = robot.get_joint_axis(joint)

        # record the kinematic parameters
        xyzs.append(xyz)
        rpys.append(rpy)
        axes.append(axis)
    print("xyz, rpy, axis = {0}, {1} ,{2}".format(xyzs, rpys, axes))

    Nb = len(joints_list_r)-1
    return Nb, xyzs, rpys, axes



def find_dyn_parm_deps(dof, parm_num, regressor_func):
    '''
    Find dynamic parameter dependencies (i.e., regressor column dependencies).
    '''

    samples = 10000
    round = 10

    pi = np.pi

    Z = np.zeros((dof * samples, parm_num))

    for i in range(samples):
        q = [float(np.random.random() * 2.0 * pi - pi) for j in range(dof)]
        dq = [float(np.random.random() * 2.0 * pi - pi) for j in range(dof)]
        ddq = [float(np.random.random() * 2.0 * pi - pi)
            for j in range(dof)]
        Z[i * dof: i * dof + dof, :] = np.matrix(
            regressor_func(q, dq, ddq)).reshape(dof, parm_num)

    R1_diag = np.linalg.qr(Z, mode='r').diagonal().round(round)
    dbi = []
    ddi = []
    for i, e in enumerate(R1_diag):
        if e != 0:
            dbi.append(i)
        else:
            ddi.append(i)
    dbn = len(dbi)

    P = np.mat(np.eye(parm_num))[:, dbi + ddi]
    Pb = P[:, :dbn]
    Pd = P[:, dbn:]

    Rbd1 = np.mat(np.linalg.qr(Z * P, mode='r'))
    Rb1 = Rbd1[:dbn, :dbn]
    Rd1 = Rbd1[:dbn, dbn:]

    Kd = np.mat((np.linalg.inv(Rb1) * Rd1).round(round))

    return Pb, Pd, Kd

class FourierSeries():
    def __init__(self, Rank = 5, channel = 7,
                 bias=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                 ff = 0.01) -> None:
        assert channel == len(bias), "Different size of Rank and bias!"
        
        # self.a = cs.SX.sym('a', Rank,channel)
        # self.b = cs.SX.sym('b', Rank,channel)
        self.Rank = Rank
        self.bias = bias
        self.ff =ff
        self.channel = channel


    # def value(self, t):
    #     q = self.bias
    #     for i in range(self.channel):
    #         for l in range(self.Rank):
    #             wl = (l * self.ff* math.pi* 2.0) 
    #             q[i] = q[i] + self.a[l,i]/wl * cs.sin(wl * t) 
    #             - self.b[l,i] * cs.cos(wl * t)

    #     return q
    
    def FourierFunction(self, t, a, b, name):
        q = copy.deepcopy(self.bias)
        for i in range(self.channel):
            for l in range(self.Rank):
                wl = ((l+1) * self.ff* math.pi* 2.0) 
                q[i] = q[i] + a[l,i]/wl * cs.sin(wl * t) 
                - b[l,i] * cs.cos(wl * t)

        return cs.Function(name, [a, b, t], q)
    
    def FourierValue(self, a,b,t):
        q = copy.deepcopy(self.bias)
        for i in range(self.channel):
            for l in range(self.Rank):
                wl = ((l+1) * self.ff* math.pi* 2.0) 
                q[i] = q[i] + a[l,i]/wl * np.sin(wl * t) 
                - b[l,i] * np.cos(wl * t)

        return q







class Estimator(Node):
    def __init__(self, node_name = "para_estimatior", dt_ = 5.0, N_ = 100) -> None:
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
        self.N = N_

        # self.lbr_command_timer_ = self.create_timer(self.dt_, self.timer_cb_regressor)

        # 1. Get the kinematic parameters of every joints
        self.robot = optas.RobotModel(
            xacro_filename=path,
            time_derivs=[1],  # i.e. joint velocity
        )
        # root = self.robot.urdf.get_root()
        # ee_link = "lbr_link_ee"
        # xyzs, rpys, axes = [], [], []


        # joints_list = self.robot.urdf.get_chain(root, ee_link, links=False)
        # print("joints_list = {0}"
        #       .format(joints_list)
        #       )
        # # assumption: The first joint is fixed. The information in this joint is not recorded
        # """
        # xyzs starts from joint 0 to joint ee
        # rpys starts from joint 0 to joint ee
        # axes starts from joint 0 to joint ee
        # """
        # joints_list_r = joints_list[1:]
        # for joint_name in joints_list_r:
        #     print(joint_name)
        #     joint = self.robot.urdf.joint_map[joint_name]
        #     xyz, rpy = self.robot.get_joint_origin(joint)
        #     axis = self.robot.get_joint_axis(joint)

        #     # record the kinematic parameters
        #     xyzs.append(xyz)
        #     rpys.append(rpy)
        #     axes.append(axis)
        # print("xyz, rpy, axis = {0}, {1} ,{2}".format(xyzs, rpys, axes))
        
        # Nb = len(joints_list_r)-1

        Nb, xyzs, rpys, axes = getJointParametersfromURDF(self.robot)
        self.dynamics_ = RNEA_function(Nb,1,rpys,xyzs,axes)
        self.Ymat, self.PIvector = DynamicLinearlization(self.dynamics_,Nb)


        urdf_string_ = xacro.process(path)
        robot = urdf.URDF.from_xml_string(urdf_string_)

        # print([joint.name for joint in robot.joints if joint.origin is not None])
        # print([joint.origin.xyz for joint in robot.joints if joint.origin is not None])

        # print([link.name for link in robot.links if link.inertial is not None])
        # print([link.inertial.origin.xyz for link in robot.links if link.inertial is not None])
        # print([link.inertial.mass for link in robot.links if link.inertial is not None])

        masses = [link.inertial.mass for link in robot.links if link.inertial is not None]#+[1.0]
        self.masses_np = np.array(masses[1:])
        # print("masses = {0}".format(self.masses_np))

        massesCenter = [link.inertial.origin.xyz for link in robot.links if link.inertial is not None]#+[[0.0,0.0,0.0]]
        self.massesCenter_np = np.array(massesCenter[1:]).T
        # Inertia = [np.mat(link.inertial.inertia.to_matrix()) for link in robot.links if link.inertial is not None]
        Inertia = [link.inertial.inertia.to_matrix() for link in robot.links if link.inertial is not None]
        self.Inertia_np = np.hstack(tuple(Inertia[1:]))
        
       
    
    @ staticmethod
    def readCsvToList(path):
        l = []
        with open(path) as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                joint_names = [x.strip() for x in list(row.keys())]
                l.append([float(x) for x in row.values()])
        return l    
    

    def ExtractFromCsv(self):
        path_pos = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "joint_states_positions.csv",
        )

        path_vel = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "joint_states_velocities.csv",
        )

        path_eff = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "joint_states_efforts.csv",
        )
        pos = Estimator.readCsvToList(path_pos)
        vel = Estimator.readCsvToList(path_vel)
        eff = Estimator.readCsvToList(path_eff)

        # print("pos = {0}".format(pos))
        
        return pos, vel, eff
    
    def ExtractFromMeasurmentCsv(self):
        path_pos = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "measurements_with_ext_tau.csv",
        )

        # path_vel = os.path.join(
        #     get_package_share_directory("gravity_compensation"),
        #     "test",
        #     "joint_states_velocities.csv",
        # )

        # path_eff = os.path.join(
        #     get_package_share_directory("gravity_compensation"),
        #     "test",
        #     "joint_states_efforts.csv",
        # )
        dt = 0.01
        pos_l = []
        tau_ext_l = []
        with open(path_pos) as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                # print("111 = {0}".format(row.values()))
                pl = list(row.values())[0:7]
                tl = list(row.values())[7:14]
                joint_names = [x.strip() for x in list(row.keys())]
                pos_l.append([float(x) for x in pl])
                tau_ext_l.append([float(x) for x in tl])

        vel_l =[]
        filter = TD_2order(T=0.01)
        for id in range(len(pos_l)):
            if id == 0:
                vel_l.append([0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0])
            else:
                vel_l.append([(p-p_1)/dt for (p,p_1) in zip(pos_l[id],pos_l[id-1])])



        return pos_l,vel_l,tau_ext_l    
        
    
    def generate_opt_traj(self,Ff, sampling_rate, Rank=5, 
                          q_min=-20.0*np.ones(7), q_max =20.0*np.ones(7),
                          q_vmin=-10.0*np.ones(7),q_vmax=10.0*np.ones(7)):

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)

        
        # sampling_rate = 0.1
        pointsNum = int(sampling_rate/(Ff*4))

        fourierInstance = FourierSeries(ff = Ff)

        a = cs.SX.sym('a', 5,7)
        b = cs.SX.sym('b', 5,7)
        t = cs.SX.sym('t', 1)

        fourierF = fourierInstance.FourierFunction(t, a, b,'f1')
        fourier = fourierF(a,b,t)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        print(fourierDot)

        Y_ = []
        Y_fri = []
        for k in range(pointsNum):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            tc = 1.0/(sampling_rate) * k
            q_list = [optas.substitute(id, t, tc) for id in fourier]#fourier(a,b,tc)
            qd_list = [optas.substitute(id, t, tc) for id in fourierDot] #fourierDot(a,b,tc)
            qdd_list = [optas.substitute(id, t, tc) for id in fourierDDot]#fourierDDot(a,b,tc)
            q = cs.vertcat(*q_list)
            qd = cs.vertcat(*qd_list)
            qdd = cs.vertcat(*qdd_list)


            Y_temp = self.Ymat(q,
                               qd,
                               qdd) @Pb 
            #[cs.sign(item) for item in qd_list])
            fri_ = cs.diag(cs.sign(qd))
            fri_ = cs.horzcat(fri_,  cs.diag(qd))
            # fri_ = [[np.sign(v), v] for v in qd_np]
            
            Y_.append(Y_temp)
            # q_nps.append(q_np)
            # qd_nps.append(qd_np)
            # qdd_nps.append(qdd_np)
            # taus.append(tau_ext)
            Y_fri.append(fri_)

        Y_r = optas.vertcat(*Y_)
        Y_fri1 = optas.vertcat(*Y_fri)

        Y = cs.horzcat(Y_r, Y_fri1)

        # print(Y)
        a_eq1 = [0.0]*7
        a_eq2 = [0.0]*7
        b_eq1 = [0.0]*7
        ab_sq_ineq1 = [0.0]*7
        ab_sq_ineq2 = [0.0]*7
        ab_sq_ineq3 = []

        lbg1 = []
        lbg2 = []
        lbg3 = []
        lbg4 = []
        lbg5 = []
        lbg6 = []
        

        ubg1 = []
        ubg2 = []
        ubg3 = []
        ubg4 = []
        ubg5 = []
        ubg6 = []
        # ab_sq_ineq4 = []
        for i in range(7):
            for l in range(5):
                print("iter {0}, {1}".format(i, l))
                a_eq1[i] = a_eq1[i] + a[l,i]/(l+1)
                b_eq1[i] = b_eq1[i] + b[l,i]
                a_eq2[i] = a_eq1[i] + a[l,i]*(l+1)

                wl = ((l+1) * Ff* math.pi* 2.0) 
                ab_sq_ineq1[i] = (ab_sq_ineq1[i]+ 
                1.0/(wl)* cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq2[i] = (ab_sq_ineq1[i]+ 
                cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq3.append(a[l,i])
                ab_sq_ineq3.append(b[l,i])

                cpr = max((l+1)*Ff/5.0*q_min[i],q_vmin[i])
                cpr2 = min((l+1)*Ff/5.0*q_max[i],q_vmax[i])
                lbg6.append(cpr)
                lbg6.append(cpr)

                ubg6.append(cpr2)
                ubg6.append(cpr2)

            lbg1.append(0.0)
            lbg2.append(0.0)
            lbg3.append(0.0)
            lbg4.append(0.0)
            lbg5.append(0.0)

            ubg1.append(0.0)
            ubg2.append(0.0)
            ubg3.append(0.0)
            ubg4.append(q_max[i])
            ubg5.append(q_vmax[i])

            # lbg.append(0.0)
            # lbg.append(0.0)
            # lbg.append(0.0)
            # lbg.append(0.0)
            # lbg.append(0.0)



        g = cs.vertcat(*(a_eq1+  a_eq2+  b_eq1+  ab_sq_ineq1+ ab_sq_ineq2 + ab_sq_ineq3))
        lbg = cs.vertcat(*(lbg1,lbg2,lbg3,lbg4,lbg5,lbg6))
        ubg = cs.vertcat(*(ubg1,ubg2,ubg3,ubg4,ubg5,ubg6))

        # print("sol['x'] = {0}".format(sol['x']),flush= True)
        A = Y.T @ Y

        print("A = {0}".format(A.shape))
        print("Y = {0}".format(Y.shape))
        # raise ValueError("Run to here")

        A_fun = optas.Function('A_fun',[a,b],[A])

        shape = A.shape[0]
        # f = cs.fmax(*A)
        U, V = find_eigen_value(7,5,A_fun,shape)

        print("U = {0}\n V = {1}".format(U,V))

        # def objective(a,b):
        #     # a,b = cs.vertsplit(cs.reshape(x,(10,7)),5)
        #     # print("Run to here")
        #     A_mat = A_fun(a,b)
        #     # print("Run to here11")
        #     # print(A_mat)
        #     # U, s, V = np.linalg.svd(cs.DM(A_mat).full())
        #     # return U.T @ A_mat @ V.T
        #     return cs.trace(A_mat)

        # raise ValueError("Run to here")

        # A_reform = U.T @ A @ V.T



        # f = -0.001*cs.norm_2(cs.norm_1(A_reform[0,0]/(A_reform[-1,-1]+0.01)))
        # f = -1.0*cs.norm_1(A_reform[0,0])
        # f = -1.0*cs.trace(A)/(A_reform[-1,-1]+0.1)
        A_inv = cs.inv(A)
        f = -1.0*cs.trace(A)*cs.trace(A_inv)
        x = cs.reshape(cs.vertcat(a,b),(1, 70))
        # fout = objective(a,b)
        # print("Run to here11")
        # f = cs.Function('f', [a,b], [fout])
        # print("x = {0}".format(x))
        # print("a = {0}, b ={1}".format(a,b))
        # print(" xx= {0},  {1}".format(x_split1,x_split2))
        problem = {'x': x,'f':f, 'g': g}
        # S = cs.qpsol('solver', 'qpoases', problem)

        # print("Run to here22")
        
        S = cs.nlpsol('S', 'ipopt', problem,{'ipopt':{'max_iter':1500 }, 'verbose':True})
        # random.random (size= (3,4))
        sol = S(x0 = 0.5* np.random.random (size= (1,70)),lbg = lbg, ubg = ubg)
        # sol = S(x0 = 0.1*np.ones([1,70]),lbg = lbg, ubg = ubg)

        print("Run to here 33")
        x_split1,x_split2 = cs.vertsplit(cs.reshape(sol['x'],(10,7)),5)
        # print("sol['x'] = {0}".format(sol['x']),flush= True)

        print("sol = {0}".format(sol['x']))

        return x_split1.full(),x_split2.full()
        # return sol['x']

    def generateToCsv(self, a, b,Ff, sampling_rate):
        
        assert a.shape == b.shape

        path1 = "/tmp/target_joint_states.csv"

        fourierInstance1 = FourierSeries(ff = Ff)

        cs_a = cs.SX.sym('ca', 5,7)
        cs_b = cs.SX.sym('cb', 5,7)
        t = cs.SX.sym('tt', 1)

        fourierF = fourierInstance1.FourierFunction(t, cs_a, cs_b,'f2')

        fourier = fourierF(cs_a,cs_b,t)

        # _f = optas.Function('fun',[t],_fourier)



        # fourier = [_fourier[i] for i in range(len(_fourier))]
        # _f = optas.Function('fun',[cs_a,cs_b,t],fourier)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        _fDot = optas.Function('fund',[cs_a,cs_b,t],fourierDot)
        # fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        pointsNum = int(sampling_rate/Ff)
        Ts = 1.0/Ff

        keys = ["lbr_joint_0", "lbr_joint_1", "lbr_joint_2", "lbr_joint_3", "lbr_joint_4", "lbr_joint_5", "lbr_joint_6",
                "lbr_joint_0v", "lbr_joint_1v", "lbr_joint_2v", "lbr_joint_3v", "lbr_joint_4v", "lbr_joint_5v", "lbr_joint_6v"]
        keys = ["time_stamps"] + keys
        values_list = []
        for k in range(pointsNum):
            tc = 1.0/(sampling_rate) * k
            # f = fourierF(np.asarray(a),np.asarray(b),tc)
            # f = _f(tc)
            # print("b = {0}".format(b))
            f_temp =fourierInstance1.FourierValue(a,b,tc)
            # print("f_temp = {0}".format(f_temp))
            fd_temp=_fDot(np.asarray(a),np.asarray(b),tc)
            
            q_list = [float(id) for id in f_temp]#fourier(a,b,tc)
            qd_list = [float(id) for id in fd_temp] #fourierDot(a,b,tc)

            # print("q_list = {0}".format(q_list))
            # print("qd_list = {0}".format(qd_list))


            values_list.append([tc]+q_list + qd_list)
            # if os.path.isfile(path1):
        with open(path1,"w") as csv_file:
            self.save_(csv_file,keys,values_list)
            # else:
            #     with open(path1,"a") as csv_file:
            #         self.save_(csv_file,keys,values_list)

        return True
    
    def save_(
        self, csv_file, keys: List[str], values_list: List[List[float]]
    ) -> None:
        # # save data to csv
        # full_path = os.path.join(path, file_name)
        # self.get_logger().info(f"Saving to {full_path}...")
        # with open(full_path, "w") as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=keys)

        csv_writer.writeheader()
        # for values in values_list:
        for values in values_list:
            csv_writer.writerow({key: value for key, value in zip(keys,values)})


    

    def timer_cb_regressor(self, positions, velocities, efforts):
        
        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        q_nps = []
        qd_nps = []
        qdd_nps = []
        taus = []
        Y_ = []
        Y_fri = []
        init_para = np.random.uniform(0.0, 0.1, size=50)
        
        for k in range(0,len(positions),1):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            q_np = [positions[k][i] for i in Order]
            # print("velocities[k] = {0}".format(velocities[k]))
            qd_np = [velocities[k][i] for i in Order]
            tau_ext = [efforts[k][i] for i in Order]

            qdlast_np = [velocities[k-1][i] for i in Order]
            # qdd_np = (np.array(qd_np)-np.array(qdlast_np))/(velocities[k][0]-velocities[k-1][0])
            filter_list = [TD_2order(T=0.01) for i in range(len(qd_np))]
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
            qdd_np = qdd_np.tolist()
            # qdd_np = [f(qd_np[id])[1] for id,f in enumerate(filter_list)]
            
            # qd_np = np.random.uniform(-0.2, 0.2, size=7)
            # qdd_np = np.random.uniform(-0.1, 0.1, size=7)

            # tau_ext = self.robot.rnea(q_np,qd_np,qdd_np)
            

            Y_temp = self.Ymat(q_np,
                               qd_np,
                               qdd_np) @Pb 
            fri_ = np.diag([float(np.sign(item)) for item in qd_np])
            fri_ = np.hstack((fri_,  np.diag(qd_np)))
            # fri_ = [[np.sign(v), v] for v in qd_np]
            
            Y_.append(Y_temp)
            # q_nps.append(q_np)
            # qd_nps.append(qd_np)
            # qdd_nps.append(qdd_np)
            taus.append(tau_ext)
            Y_fri.append(np.asarray(fri_))
            
            # print(qdd_np)

        
        Y_r = optas.vertcat(*Y_)
        # q_nps1 = np.hstack(q_nps)
        # qd_nps1 = np.hstack(qd_nps)
        # qdd_nps1 = np.hstack(qdd_nps)
        taus1 = np.hstack(taus)
        Y_fri1 = np.vstack(Y_fri)
        print("Y_fri1 = {0}".format(Y_fri1))
        print("Y_fri1 = {0}".format(Y_fri1.shape))
        print("Y_r = {0}".format(Y_r.shape))
        print("Pb = {0}".format(Pb.shape))
        pa_size = Y_r.shape[1]
        
        # solution=self.regress(init_para, q_nps1,qd_nps1,qdd_nps1,taus1)
        # print("solution = {0}".format(solution[f"{self.pam_name}/y"]))

        # print("taus1 size = {0}".format(taus1.shape))
        # print("q_nps1 size = {0}".format(q_nps1.shape))
        # print("qd_nps1 size = {0}".format(qd_nps1.shape))


        taus1 = taus1.T


        estimate_pam = np.linalg.inv(Y_r.T @ Y_r) @ Y_r.T @ taus1

        #opti = cs.Opti()
        # estimate_cs = cs.SX.sym('para', 50+14)

        Y = cs.DM(np.hstack((Y_r, Y_fri1)))
        estimate_pam = np.linalg.inv(Y.T @ Y) @ Y.T @ taus1
        # para_friction = cs.SX.sym('para', 14)
        

        estimate_cs = cs.SX.sym('para', pa_size+14)
        obj = cs.sumsqr(taus1 - Y @ estimate_cs)
        # obj = cs.sumsqr(taus1 )

        ref_pam = K @ self.PIvector(self.masses_np,self.massesCenter_np,self.Inertia_np)
        
        # lb = 0.5*ref_pam
        # ub = 1.5*ref_pam

        lb = 0.3*ref_pam
        ub = -0.3*ref_pam

        ineq_constr = [estimate_cs[i] >= lb[i] for i in range(pa_size)] + [estimate_cs[i] <= ub[i] for i in range(pa_size)]

        problem = {'x': estimate_cs, 'f': obj, 'g': cs.vertcat(*ineq_constr)}
        solver = cs.qpsol('solver', 'qpoases', problem)
        # solver = cs.nlpsol('S', 'ipopt', problem,{'ipopt':{'max_iter':1500 }, 'verbose':True})
        print("solver = {0}".format(solver))
        sol = solver()

        print("sol = {0}".format(sol['x']))

        return sol['x']
    
    def testWithEstimatedPara(self, positions, velocities, efforts, para)->None:

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        for k in range(1,len(positions),1):
            # q_np = positions[k][4,1,2,3,5,6,7]
            # qd_np = velocities[k][4,1,2,3,5,6,7]
            # tau_ext = efforts[k][4,1,2,3,5,6,7]
            # qdd_np = (np.array(velocities[k][4,1,2,3,5,6,7])-np.array(velocities[k-1][4,1,2,3,5,6,7]))/(velocities[k][0]-velocities[k-1][0])
            # qdd_np = qdd_np.tolist()

            q_np = [positions[k][i] for i in Order]
            qd_np = [velocities[k][i] for i in Order]
            tau_ext = [efforts[k][i] for i in Order]

            qdlast_np = [velocities[k-1][i] for i in Order]
            # qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01#(velocities[k][0]-velocities[k-1][0])
            # qdd_np = qdd_np.tolist()
            filter_list = [TD_2order(T=0.01) for i in range(len(qd_np))]
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
            qdd_np = qdd_np.tolist()
            # qdd_np = [f(qd_np[id])[1] for id,f in enumerate(filter_list)]

            # tau_ext = self.robot.rnea(q_np,qd_np,qdd_np)
            # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @ (solution[f"{self.pam_name}/y"] -  K @real_pam)
            # print("error = {0}".format(e))

            # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @  para - tau_ext 
            pa_size = Pb.shape[1]
            e=(self.Ymat(q_np,qd_np,qdd_np)@Pb @  para[:pa_size] + 
                np.diag(np.sign(qd_np)) @ para[pa_size:pa_size+7]+ 
                np.diag(qd_np) @ para[pa_size+7:]) - tau_ext 
            print("error1 = {0}".format(e))
            print("tau_ext = {0}".format(tau_ext))


    def saveEstimatedPara(self, parac)->None:

        path1 = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "DynamicParameters.csv",
        )

        para = parac.toarray().flatten()
        # Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        # K = Pb.T +Kd @Pd.T
        keys = ["para_{0}".format(idx) for idx in range(len(para))]
        with open(path1,"w") as csv_file:
            self.save_(csv_file,keys,[para])
        

        # for k in range(1,len(positions),1):
        #     # q_np = positions[k][4,1,2,3,5,6,7]
        #     # qd_np = velocities[k][4,1,2,3,5,6,7]
        #     # tau_ext = efforts[k][4,1,2,3,5,6,7]
        #     # qdd_np = (np.array(velocities[k][4,1,2,3,5,6,7])-np.array(velocities[k-1][4,1,2,3,5,6,7]))/(velocities[k][0]-velocities[k-1][0])
        #     # qdd_np = qdd_np.tolist()

        #     q_np = [positions[k][i] for i in Order]
        #     qd_np = [velocities[k][i] for i in Order]
        #     tau_ext = [efforts[k][i] for i in Order]

        #     qdlast_np = [velocities[k-1][i] for i in Order]
        #     # qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01#(velocities[k][0]-velocities[k-1][0])
        #     # qdd_np = qdd_np.tolist()
        #     filter_list = [TD_2order(T=0.01) for i in range(len(qd_np))]
        #     qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
        #     qdd_np = qdd_np.tolist()
        #     # qdd_np = [f(qd_np[id])[1] for id,f in enumerate(filter_list)]

        #     # tau_ext = self.robot.rnea(q_np,qd_np,qdd_np)
        #     # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @ (solution[f"{self.pam_name}/y"] -  K @real_pam)
        #     # print("error = {0}".format(e))

        #     # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @  para - tau_ext 
            # e=(self.Ymat(q_np,qd_np,qdd_np)@Pb @  para[:50] + 
            #     np.diag(np.sign(qd_np)) @ para[50:57]+ 
            #     np.diag(qd_np) @ para[57:]) - tau_ext 
            # print("error1 = {0}".format(e))
            # print("tau_ext = {0}".format(tau_ext))

        # print("taus1 size = {0}".format(taus1.shape))
        # print("q_nps1 size = {0}".format(q_nps1.shape))
        # print("qd_nps1 size = {0}".format(qd_nps1.shape))

        # real_pam=self.PIvector(self.masses_np,self.massesCenter_np,self.Inertia_np)







def main(args=None):
    rclpy.init(args=args)
    paraEstimator = Estimator()
    Ff = 0.01
    sampling_rate = 1.0
    a,b = paraEstimator.generate_opt_traj(Ff = Ff,sampling_rate = sampling_rate)
    print("a = {0} \n b = {1}".format(a,b))
    # a, b = np.ones([5,7]),np.ones([5,7])
    # print("a = {0} \n b = {1}".format(type(a),type(b)))

    ret = paraEstimator.generateToCsv(a,b,Ff = Ff,sampling_rate=sampling_rate*100.0)
    if ret:
        print("Done! Congratulations! ")



    # positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentCsv()
    # estimate_pam = paraEstimator.timer_cb_regressor(positions, velocities, efforts)
    # print("estimate_pam = {0}".format(estimate_pam))
    # paraEstimator.testWithEstimatedPara(positions, velocities, efforts,estimate_pam)
    # paraEstimator.saveEstimatedPara(estimate_pam)

    rclpy.shutdown()



if __name__ == "__main__":
    main()