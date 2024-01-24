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
from convexhallExtraction import get_convex_hull
import random

from IDmodel import TD_2order, find_dyn_parm_deps, RNEA_function,DynamicLinearlization,getJointParametersfromURDF


def getConstraintsinJointSpace(robot,point_coord = [0.]*3,Nb=7, base_link="link_3", base_joint_name="A3", ee_link="link_ee"):
    q = cs.SX.sym('q', Nb, 1)

    pe = robot.get_global_link_position(ee_link, q)
    Re = robot.get_global_link_rotation(ee_link, q)

    pb = robot.get_global_link_position(base_link, q)
    Rb = robot.get_global_link_rotation(base_link, q)

    pp = pe + Re[:,0]*point_coord[0] + Re[:,1]*point_coord[1] + Re[:,2]*point_coord[2]

    robot_urdf = robot.urdf
    joint = robot_urdf.joint_map[base_joint_name]
    xyz, _ = robot.get_joint_origin(joint)

    print("robot_urdf.joint_map = ",robot_urdf.joint_map)

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
    
    def FourierFunction(self, t, a, b, name):
        q = copy.deepcopy(self.bias)
        for i in range(self.channel):
            for l in range(self.Rank):
                wl = ((l+1) * self.ff* math.pi* 2.0) 
                q[i] = q[i] + a[l,i]/wl * cs.sin(wl * t) 
                - b[l,i]/wl * cs.cos(wl * t)

        return cs.Function(name, [a, b, t], q)
    
    def FourierValue(self, a,b,t):
        q = copy.deepcopy(self.bias)
        for i in range(self.channel):
            for l in range(self.Rank):
                wl = ((l+1) * self.ff* math.pi* 2.0) 
                q[i] = q[i] + a[l,i]/wl * np.sin(wl * t) 
                - b[l,i]/wl * np.cos(wl * t)

        return q


class TrajGeneration(Node):
    def __init__(self, node_name = "para_estimatior", dt_ = 5.0, N_ = 100,gravity_vector=[4.905, 0.0, -8.496]) -> None:
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
        gv = gravity_vector

        # 1. Get the kinematic parameters of every joints
        self.robot = optas.RobotModel(
            xacro_filename=path,
            time_derivs=[1],  # i.e. joint velocity
        )


        Nb, xyzs, rpys, axes = getJointParametersfromURDF(self.robot)
        self.dynamics_ = RNEA_function(Nb,1,rpys,xyzs,axes,gravity_para=cs.DM(gv))
        self.Ymat, self.PIvector = DynamicLinearlization(self.dynamics_,Nb)


        urdf_string_ = xacro.process(path)
        robot = urdf.URDF.from_xml_string(urdf_string_)


        masses = [link.inertial.mass for link in robot.links if link.inertial is not None]#+[1.0]
        self.masses_np = np.array(masses[1:])


        massesCenter = [link.inertial.origin.xyz for link in robot.links if link.inertial is not None]#+[[0.0,0.0,0.0]]
        self.massesCenter_np = np.array(massesCenter[1:]).T
        
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

    
    def ExtractFromMeasurmentCsv(self):
        path_pos = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "measurements_with_ext_tau.csv",
        )

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
    
    def generate_opt_traj_Link(self,Ff, sampling_rate, Rank=5, 
                          q_min=-2.0*np.ones(7), q_max =2.0*np.ones(7),
                          q_vmin=-8.0*np.ones(7),q_vmax=8.0*np.ones(7),
                          f_path = None, g_path=None,bias=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        
        # sampling_rate = 0.1
        pointsNum = int(sampling_rate/(Ff))
        print("pointsNum",pointsNum)
        # raise ValueError("run to here")

        fourierInstance = FourierSeries(ff = Ff,bias=bias)

        a = cs.SX.sym('a', Rank,7)
        b = cs.SX.sym('b', Rank,7)
        t = cs.SX.sym('t', 1)

        fourierF = fourierInstance.FourierFunction(t, a, b,'f1')
        fourier = fourierF(a,b,t)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        print(fourierDot)

        path_pos = os.path.join(
                get_package_share_directory("med7_dock_description"),
                "meshes",
                "EndEffector.STL",
            )

        points = get_convex_hull(path_pos)
        # print("points", points)
        # raise Exception("Run to here")
        str_prefix = "lbr_"
        vfs_fun = []

        for point in points:
            for i in range(2,6):
                vfs_fun.append(getConstraintsinJointSpace(self.robot, point_coord=point, 
                                           base_link="link_"+str(i),
                                           base_joint_name="A"+str(i)
                                           ))


        Y_ = []
        Y_fri = []
        pfun_list = []
        for k in range(pointsNum):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            tc = 1.0/(sampling_rate) * k
            print("tc = ",tc)
            
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
            

            for j in range(len(vfs_fun)):
                pfun_list.append(vfs_fun[j](q))


            Y_.append(Y_temp)
            Y_fri.append(fri_)

        Y_r = optas.vertcat(*Y_)
        Y_fri1 = optas.vertcat(*Y_fri)

        Y = Y_r

        # Y = cs.horzcat(Y_r, Y_fri1)

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
                a_eq2[i] = a_eq2[i] + a[l,i]*(l+1)

                wl = ((l+1) * Ff* math.pi* 2.0) 
                ab_sq_ineq1[i] = (ab_sq_ineq1[i]+ 
                1.0/(wl)* cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq2[i] = (ab_sq_ineq2[i]+ 
                cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq3.append(a[l,i])
                ab_sq_ineq3.append(b[l,i])


                cpr2 = min((l+1)*Ff/5.0*2.0*math.pi*q_max[i],q_vmax[i])
                cpr = max((l+1)*Ff/5.0*2.0*math.pi*q_min[i],q_vmin[i])
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

        g = cs.vertcat(*(a_eq1+  a_eq2+  b_eq1+  ab_sq_ineq1+ ab_sq_ineq2 + ab_sq_ineq3 +pfun_list))
        lbg = cs.vertcat(*(lbg1,lbg2,lbg3,lbg4,lbg5,lbg6, [0.0]*len(pfun_list)))
        ubg = cs.vertcat(*(ubg1,ubg2,ubg3,ubg4,ubg5,ubg6, [1e10]*len(pfun_list)))


        A = Y.T @ Y
        # print("Y = {0}".format(Y.shape))
        A_inv = cs.inv(A)

        f1 = cs.simplify(1.0*cs.norm_fro(A) * cs.norm_fro(A_inv))
        f = cs.simplify(1.0*cs.norm_fro(A) + cs.norm_fro(A_inv))
        x = cs.reshape(cs.vertcat(a,b),(1, 2*Rank*7))
        # fout = objective(a,b)
        

        fc = optas.Function('fc',[a,b],[A])
        f_fun = optas.Function('ff',[a,b],[f])
        # _f_fun = optas.Function('f_ffc',[a,b],[_f])
        g_fun = optas.Function('gf',[a,b],[g])

        G_max = 1# 
        values_f_min = 10e10
        eps = 0.03

        init_x0_best = -eps*np.ones((1,2*Rank*7)) +  2* eps* np.random.random (size= (1,2*Rank*7))
        reject_sample = 100

        problem = {'x': x,'f':f, 'g': g}
        S = cs.nlpsol('S', 'ipopt', problem,
                      {'ipopt':{'max_iter':50000 }, 
                       'verbose':False,
                       "ipopt.hessian_approximation":"limited-memory"
                       })
        
        problem1 = {'x': x,'f':f1, 'g': g}
        S1 = cs.nlpsol('S', 'ipopt', problem1,
                      {'ipopt':{'max_iter':50000 }, 
                       'verbose':False,
                       "ipopt.hessian_approximation":"limited-memory"
                       })

        for iter in range(G_max):
            for num in range(reject_sample):
                x_sample_temp = eps* np.random.random (size= (1,2*Rank*7))
                init_x0 = copy.deepcopy(x_sample_temp)
                a_init, b_init =  np.split(x_sample_temp.reshape(2*Rank,7),2)
                g_data = g_fun(a_init, b_init)

                if(np.all(g_data < ubg) and np.all(g_data > lbg)):
                    print("Find a initial solution here")
                    break
            init_x0 = copy.deepcopy(x_sample_temp)
            a_init, b_init =  np.split(x_sample_temp.reshape(2*Rank,7),2)

            for k in range(1):
                sol = S(x0 = init_x0,lbg = lbg, ubg = ubg)
                # sol = S1(x0 = sol['x'],lbg = lbg, ubg = ubg)
                init_x0 = sol['x']

            a_, b_ =  cs.vertsplit(cs.reshape(sol['x'],(2*Rank,7)),Rank)
            # values_f = f_fun(a_, b_)

            eigenvalues, eigenvectors = np.linalg.eig(fc(a_,b_))

            print("fc = ",eigenvalues)
            print("a = {0} \n b = {1}".format(a,b))
            values_f = np.sqrt(eigenvalues[0]/eigenvalues[-1])


            if values_f_min > values_f:

                print(" find a better value = {0}".format(values_f))
                _x0_best = sol['x']
                values_f_min = values_f
                if (values_f < 1000):
                    break

        x_split1,x_split2 = cs.vertsplit(cs.reshape(_x0_best,(2*Rank,7)),Rank)

        print("sol = {0}".format(_x0_best))
        return x_split1.full(),x_split2.full(),fc
    
    def get_optimization_problem(self,Ff, sampling_rate, Rank=5, 
                          q_min=-3.0*np.ones(7), q_max =3.0*np.ones(7),
                          q_vmin=-6.0*np.ones(7),q_vmax=6.0*np.ones(7),
                          f_path = None, g_path=None,bias=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        
        # sampling_rate = 0.1
        pointsNum = int(sampling_rate/(Ff))
        print("pointsNum",pointsNum)
        # raise ValueError("run to here")

        fourierInstance = FourierSeries(ff = Ff,bias=bias)

        a = cs.SX.sym('a', Rank,7)
        b = cs.SX.sym('b', Rank,7)
        t = cs.SX.sym('t', 1)

        fourierF = fourierInstance.FourierFunction(t, a, b,'f1')
        fourier = fourierF(a,b,t)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        print(fourierDot)

        path_pos = os.path.join(
                get_package_share_directory("med7_dock_description"),
                "meshes",
                "EndEffector.STL",
            )

        points = get_convex_hull(path_pos)
        # print("points", points)
        # raise Exception("Run to here")
        str_prefix = "lbr_"
        vfs_fun = []

        for point in points:
            for i in range(2,6):
                vfs_fun.append(getConstraintsinJointSpace(self.robot, point_coord=point, 
                                           base_link="link_"+str(i),
                                           base_joint_name="A"+str(i)
                                           ))


        Y_ = []
        Y_fri = []
        pfun_list = []
        for k in range(pointsNum):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            tc = 1.0/(sampling_rate) * k
            print("tc = ",tc)
            
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
            

            for j in range(len(vfs_fun)):
                pfun_list.append(vfs_fun[j](q))


            Y_.append(Y_temp)
            Y_fri.append(fri_)

        Y_r = optas.vertcat(*Y_)
        Y_fri1 = optas.vertcat(*Y_fri)

        Y = Y_r

        # Y = cs.horzcat(Y_r, Y_fri1)

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
                a_eq2[i] = a_eq2[i] + a[l,i]*(l+1)

                wl = ((l+1) * Ff* math.pi* 2.0) 
                ab_sq_ineq1[i] = (ab_sq_ineq1[i]+ 
                1.0/(wl)* cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq2[i] = (ab_sq_ineq2[i]+ 
                cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq3.append(a[l,i])
                ab_sq_ineq3.append(b[l,i])


                cpr2 = min((l+1)*Ff/5.0*2.0*math.pi*q_max[i],q_vmax[i])
                cpr = max((l+1)*Ff/5.0*2.0*math.pi*q_min[i],q_vmin[i])
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
            ubg4.append(Ff* math.pi* 2.0  *q_max[i])
            ubg5.append(q_vmax[i])

        g = cs.vertcat(*(a_eq1+  a_eq2+  b_eq1+  ab_sq_ineq1+ ab_sq_ineq2 + ab_sq_ineq3 +pfun_list))
        lbg = cs.vertcat(*(lbg1,lbg2,lbg3,lbg4,lbg5,lbg6, [0.0]*len(pfun_list)))
        ubg = cs.vertcat(*(ubg1,ubg2,ubg3,ubg4,ubg5,ubg6, [1e10]*len(pfun_list)))


        A = Y.T @ Y
        # print("Y = {0}".format(Y.shape))
        A_inv = cs.inv(A)

        f1 = cs.simplify(1.0*cs.norm_fro(A) * cs.norm_fro(A_inv))
        f = cs.simplify(1.0*cs.norm_fro(A) + cs.norm_fro(A_inv))
        x = cs.reshape(cs.vertcat(a,b),(1, 2*Rank*7))
        # fout = objective(a,b)
        

        fc = optas.Function('fc',[a,b],[A])
        f_fun = optas.Function('ff',[a,b],[f])
        # _f_fun = optas.Function('f_ffc',[a,b],[_f])
        g_fun = optas.Function('gf',[a,b],[g])

        G_max = 1# 
        values_f_min = 10e10
        eps = 0.03

        init_x0_best = -eps*np.ones((1,2*Rank*7)) +  2* eps* np.random.random (size= (1,2*Rank*7))
        reject_sample = 100

        problem = {'x': x,'f':f, 'g': g}
        S = cs.nlpsol('S', 'ipopt', problem,
                      {'ipopt':{'max_iter':50000 }, 
                       'verbose':False,
                       "ipopt.hessian_approximation":"limited-memory"
                       })
        
        # problem1 = {'x': x,'f':f1, 'g': g}
        # S1 = cs.nlpsol('S', 'ipopt', problem1,
        #               {'ipopt':{'max_iter':50000 }, 
        #                'verbose':False,
        #                "ipopt.hessian_approximation":"limited-memory"
        #                })
        return S,lbg,ubg,fc
    
    def find_optimal_point_with_start(self, S,lbg, ubg , Rank=5,x_sample_temp = eps* np.random.random (size= (1,70))):
        
        
        init_x0 = copy.deepcopy(x_sample_temp)
        sol = S(x0 = init_x0,lbg = lbg, ubg = ubg)
        _x0_best = sol['x']
        x_split1,x_split2 = cs.vertsplit(cs.reshape(_x0_best,(2*Rank,7)),Rank)

        # print("sol = {0}".format(_x0_best))
        return x_split1.full(),x_split2.full() 
    
    def find_optimal_point_with_randomstart(self, S,lbg, ubg , Rank=5):
        eps = 0.03
        
        x_sample_temp = eps* np.random.random (size= (1,70))

        return self.find_optimal_point_with_start(S,lbg, ubg , Rank,x_sample_temp)
    

    def trajectory_with_random(self,Rank=5):
        x_sample_temp = 0.3* np.random.random (size= (1,70))
        x_split1,x_split2 = cs.vertsplit(cs.reshape(x_sample_temp,(2*Rank,7)),Rank)
        return x_split1.full(),x_split2.full() 



        # for iter in range(G_max):
        #     for num in range(reject_sample):
        #         x_sample_temp = eps* np.random.random (size= (1,2*Rank*7))
        #         init_x0 = copy.deepcopy(x_sample_temp)
        #         a_init, b_init =  np.split(x_sample_temp.reshape(2*Rank,7),2)
        #         g_data = g_fun(a_init, b_init)

        #         if(np.all(g_data < ubg) and np.all(g_data > lbg)):
        #             print("Find a initial solution here")
        #             break
        #     init_x0 = copy.deepcopy(x_sample_temp)
        #     a_init, b_init =  np.split(x_sample_temp.reshape(2*Rank,7),2)

        #     for k in range(1):
        #         sol = S(x0 = init_x0,lbg = lbg, ubg = ubg)
        #         # sol = S1(x0 = sol['x'],lbg = lbg, ubg = ubg)
        #         init_x0 = sol['x']

        #     a_, b_ =  cs.vertsplit(cs.reshape(sol['x'],(2*Rank,7)),Rank)
        #     # values_f = f_fun(a_, b_)

        #     eigenvalues, eigenvectors = np.linalg.eig(fc(a_,b_))

        #     print("fc = ",eigenvalues)
        #     print("a = {0} \n b = {1}".format(a,b))
        #     values_f = np.sqrt(eigenvalues[0]/eigenvalues[-1])


        #     if values_f_min > values_f:

        #         print(" find a better value = {0}".format(values_f))
        #         _x0_best = sol['x']
        #         values_f_min = values_f
        #         if (values_f < 1000):
        #             break

        # x_split1,x_split2 = cs.vertsplit(cs.reshape(_x0_best,(2*Rank,7)),Rank)

        # print("sol = {0}".format(_x0_best))
        # return x_split1.full(),x_split2.full(),fc
        
    
    def generate_opt_traj(self,Ff, sampling_rate, Rank=5, 
                          q_min=-1.0*np.ones(7), q_max =3.0*np.ones(7),
                          q_vmin=-5.0*np.ones(7),q_vmax=5.0*np.ones(7),
                          f_path = None, g_path=None):

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        
        # sampling_rate = 0.1
        pointsNum = int(sampling_rate/(Ff))
        print("pointsNum",pointsNum)
        # raise ValueError("run to here")

        fourierInstance = FourierSeries(ff = Ff)

        a = cs.SX.sym('a', 5,7)
        b = cs.SX.sym('b', 5,7)
        t = cs.SX.sym('t', 1)

        fourierF = fourierInstance.FourierFunction(t, a, b,'f1')
        fourier = fourierF(a,b,t)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        print(fourierDot)

        path_pos = os.path.join(
                get_package_share_directory("med7_dock_description"),
                "meshes",
                "EndEffector.STL",
            )

        points = get_convex_hull(path_pos)
        # print("points", points)
        # raise Exception("Run to here")
        str_prefix = "lbr_"
        vfs_fun = []

        for point in points:
            for i in range(2,6):
                vfs_fun.append(getConstraintsinJointSpace(self.robot, point_coord=point, 
                                           base_link="link_"+str(i),
                                           base_joint_name="A"+str(i)
                                           ))


        Y_ = []
        Y_fri = []
        pfun_list = []
        for k in range(pointsNum):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            tc = 1.0/(sampling_rate) * k
            print("tc = ",tc)
            
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
            

            for j in range(len(vfs_fun)):
                pfun_list.append(vfs_fun[j](q))


            Y_.append(Y_temp)
            Y_fri.append(fri_)

        Y_r = optas.vertcat(*Y_)
        Y_fri1 = optas.vertcat(*Y_fri)


        # Y = Y_r

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
                a_eq2[i] = a_eq2[i] + a[l,i]*(l+1)

                wl = ((l+1) * Ff* math.pi* 2.0) 
                ab_sq_ineq1[i] = (ab_sq_ineq1[i]+ 
                1.0/(wl)* cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq2[i] = (ab_sq_ineq2[i]+ 
                cs.sqrt(a[l,i]*a[l,i] + b[l,i]*b[l,i]))

                ab_sq_ineq3.append(a[l,i])
                ab_sq_ineq3.append(b[l,i])


                cpr2 = min((l+1)*Ff/5.0*2.0*math.pi*q_max[i],q_vmax[i])
                cpr = max((l+1)*Ff/5.0*2.0*math.pi*q_min[i],q_vmin[i])
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

        g = cs.vertcat(*(a_eq1+  a_eq2+  b_eq1+  ab_sq_ineq1+ ab_sq_ineq2 + ab_sq_ineq3 +pfun_list))
        lbg = cs.vertcat(*(lbg1,lbg2,lbg3,lbg4,lbg5,lbg6, [0.0]*len(pfun_list)))
        ubg = cs.vertcat(*(ubg1,ubg2,ubg3,ubg4,ubg5,ubg6, [1e10]*len(pfun_list)))


        A = Y.T @ Y
        # print("Y = {0}".format(Y.shape))
        A_inv = cs.inv(A)

        f = cs.simplify(1.0*cs.norm_fro(A) + cs.norm_fro(A_inv))
        x = cs.reshape(cs.vertcat(a,b),(1, 70))
        # fout = objective(a,b)
        

        fc = optas.Function('fc',[a,b],[A])
        f_fun = optas.Function('ff',[a,b],[f])
        # _f_fun = optas.Function('f_ffc',[a,b],[_f])
        g_fun = optas.Function('gf',[a,b],[g])

        G_max = 3# 
        values_f_min = 10e10
        eps = 2.0

        init_x0_best = eps* np.random.random (size= (1,70))
        reject_sample = 100

        problem = {'x': x,'f':f, 'g': g}
        S = cs.nlpsol('S', 'ipopt', problem,
                      {'ipopt':{'max_iter':50 }, 
                       'verbose':False,
                       "ipopt.hessian_approximation":"limited-memory"
                       })

        for iter in range(G_max):
            for num in range(reject_sample):
                x_sample_temp = eps* np.random.random (size= (1,70))
                init_x0 = copy.deepcopy(x_sample_temp)
                a_init, b_init =  np.split(x_sample_temp.reshape(10,7),2)
                g_data = g_fun(a_init, b_init)

                if(np.all(g_data < ubg) and np.all(g_data > lbg)):
                    print("Find a initial solution here")
                    break
            init_x0 = copy.deepcopy(x_sample_temp)
            a_init, b_init =  np.split(x_sample_temp.reshape(10,7),2)

            sol = S(x0 = init_x0,lbg = lbg, ubg = ubg)
            a_, b_ =  cs.vertsplit(cs.reshape(sol['x'],(10,7)),5)
            values_f = f_fun(a_, b_)
            if values_f_min > values_f:

                print(" find a better value = {0}".format(values_f))
                _x0_best = sol['x']
                values_f_min = values_f
                if (values_f < 1000):
                    break

        x_split1,x_split2 = cs.vertsplit(cs.reshape(_x0_best,(10,7)),5)

        print("sol = {0}".format(_x0_best))
        return x_split1.full(),x_split2.full(),fc


    def generateToCsv(self, a, b,Ff, sampling_rate,path=None,scale=1.0):
        
        assert a.shape == b.shape
        if path is None:
            path1 = "/tmp/target_joint_states.csv"
        else:
            path1 = path

        values_list,keys = self.generateToList(a, b, Ff, sampling_rate)

        # fourierInstance1 = FourierSeries(ff = Ff)

        # cs_a = cs.SX.sym('ca', 5,7)
        # cs_b = cs.SX.sym('cb', 5,7)
        # t = cs.SX.sym('tt', 1)

        # fourierF = fourierInstance1.FourierFunction(t, cs_a, cs_b,'f2')

        # fourier = fourierF(cs_a,cs_b,t)

        # fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        # _fDot = optas.Function('fund',[cs_a,cs_b,t],fourierDot)
        # # fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        # pointsNum = int(sampling_rate/Ff)
        # Ts = 1.0/Ff

        # keys = ["lbr_A0", "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6",
        #         "lbr_A0v", "lbr_A1v", "lbr_A2v", "lbr_A3v", "lbr_A4v", "lbr_A5v", "lbr_A6v"]
        # keys = ["time_stamps"] + keys
        # values_list = []
        # for k in range(pointsNum):
        #     tc = 1.0/(sampling_rate) * k
        #     # f = fourierF(np.asarray(a),np.asarray(b),tc)
        #     # f = _f(tc)
        #     # print("b = {0}".format(b))
        #     f_temp =fourierInstance1.FourierValue(a,b,scale*tc)
        #     # print("f_temp = {0}".format(f_temp))
        #     fd_temp=_fDot(np.asarray(a),np.asarray(b),scale*tc)
            
        #     q_list = [float(id) for id in f_temp]#fourier(a,b,tc)
        #     qd_list = [float(id) for id in fd_temp] #fourierDot(a,b,tc)

        #     values_list.append([tc]+q_list + qd_list)
        #     # if os.path.isfile(path1):
        with open(path1,"w") as csv_file:
            self.save_(csv_file,keys,values_list)

        return True
    

    def generateToList(self, a, b,Ff, sampling_rate,scale=1.0):
        
        assert a.shape == b.shape

        fourierInstance1 = FourierSeries(ff = Ff)

        cs_a = cs.SX.sym('ca', 5,7)
        cs_b = cs.SX.sym('cb', 5,7)
        t = cs.SX.sym('tt', 1)

        fourierF = fourierInstance1.FourierFunction(t, cs_a, cs_b,'f2')

        fourier = fourierF(cs_a,cs_b,t)

        fourierDot = [optas.jacobian(fourier[i],t) for i in range(len(fourier))]
        _fDot = optas.Function('fund',[cs_a,cs_b,t],fourierDot)
        # fourierDDot = [optas.jacobian(fourierDot[i],t) for i in range(len(fourierDot))]

        pointsNum = int(sampling_rate/Ff)
        Ts = 1.0/Ff

        keys = ["lbr_A0", "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6",
                "lbr_A0v", "lbr_A1v", "lbr_A2v", "lbr_A3v", "lbr_A4v", "lbr_A5v", "lbr_A6v"]
        keys = ["time_stamps"] + keys
        values_list = []
        for k in range(pointsNum):
            tc = 1.0/(sampling_rate) * k
            # f = fourierF(np.asarray(a),np.asarray(b),tc)
            # f = _f(tc)
            # print("b = {0}".format(b))
            f_temp =fourierInstance1.FourierValue(a,b,scale*tc)
            # print("f_temp = {0}".format(f_temp))
            fd_temp=_fDot(np.asarray(a),np.asarray(b),scale*tc)
            
            q_list = [float(id) for id in f_temp]#fourier(a,b,tc)
            qd_list = [float(id) for id in fd_temp] #fourierDot(a,b,tc)

            values_list.append([tc]+q_list + qd_list)
            # if os.path.isfile(path1):
        # with open(path1,"w") as csv_file:
        #     self.save_(csv_file,keys,values_list)

        return values_list,keys
    
    def save_(
        self, csv_file, keys: List[str], values_list: List[List[float]]
    ) -> None:

        csv_writer = csv.DictWriter(csv_file, fieldnames=keys)

        csv_writer.writeheader()
        # for values in values_list:
        for values in values_list:
            csv_writer.writerow({key: value for key, value in zip(keys,values)})




    def output_perform_with_full(self,a,b,path_f):

        x = cs.reshape(cs.vertcat(a,b),(1, 70))

        _f = cs.Function.load(path_f)
        # _g = cs.Function.load(path_g)
        f = _f(a,b)
        # g = _g(a,b)
        return f

    

    def load_analyse_data(self,path_f, path_g):

        a = cs.SX.sym('a', 5,7)
        b = cs.SX.sym('b', 5,7)

        x = cs.reshape(cs.vertcat(a,b),(1, 70))

        _f = cs.Function.load(path_f)
        _g = cs.Function.load(path_g)
        f = _f(a,b)
        g = _g(a,b)

        G_max = 10
        eps = 2.0
        # reject_sample = 100
        # values_f_min = 10e10

        init_x0_best = 2.0* np.random.random (size= (1,70))

        def objective_function(pop):
            Alpha1 = 0.75
            Alpha2 = 0.25
            fitness = np.zeros(pop.shape[0])
            for i in range(pop.shape[0]):
                x = copy.deepcopy(pop[i])
                a_init, b_init =  np.split(x.reshape(10,7),2)

                fitness[i] = -Alpha1*_f(a_init, b_init) #- Alpha2*(con_a+con_b)
            return fitness
        
        def selection(pop, fitness, pop_size):
            next_generation = np.zeros((pop_size, pop.shape[1]))
            elite = np.argmax(fitness)
            # print("elite = ",elite)
            next_generation[0] = pop[elite]  # keep the best
            print("pop[elite]  = ",np.max(fitness))
            fitness = np.delete(fitness,elite)
            pop = np.delete(pop,elite,axis=0)
            P = [f/sum(fitness) for f in fitness]  # selection probability
            index = list(range(pop.shape[0]))
            index_selected = np.random.choice(index, size=pop_size-1, replace=False, p=P)
            s = 0
            for j in range(pop_size-1):
                next_generation[j+1] = pop[index_selected[s]]
                s +=1
            return next_generation
        
        def crossover(pop, crossover_rate):
            offspring = np.zeros((crossover_rate, pop.shape[1]))
            for i in range(int(crossover_rate/2)):
                r1=random.randint(0, pop.shape[0]-1)
                r2 = random.randint(0, pop.shape[0]-1)
                while r1 == r2:
                    r1 = random.randint(0, pop.shape[0]-1)
                    r2 = random.randint(0, pop.shape[0]-1)
                cutting_point = random.randint(1, pop.shape[1] - 1)
                offspring[2*i, 0:cutting_point] = pop[r1, 0:cutting_point]
                offspring[2*i, cutting_point:] = pop[r2, cutting_point:]
                offspring[2*i+1, 0:cutting_point] = pop[r2, 0:cutting_point]
                offspring[2*i+1, cutting_point:] = pop[r1, cutting_point:]
            return offspring
        
        def mutation(pop, mutation_rate):
            offspring = np.zeros((mutation_rate, pop.shape[1]))
            for i in range(int(mutation_rate/2)):
                r1=random.randint(0, pop.shape[0]-1)
                r2 = random.randint(0, pop.shape[0]-1)
                while r1 == r2:
                    r1 = random.randint(0, pop.shape[0]-1)
                    r2 = random.randint(0, pop.shape[0]-1)
                cutting_point = random.randint(0, pop.shape[1]-1)
                offspring[2*i] = pop[r1]
                offspring[2*i,cutting_point] = pop[r2,cutting_point]
                offspring[2*i+1] = pop[r2]
                offspring[2*i+1, cutting_point] = pop[r1, cutting_point]
            return offspring
        
        def local_search(pop, n_sol, step_size):
            # number of offspring chromosomes generated from the local search
            offspring = np.zeros((n_sol, pop.shape[1]))
            for i in range(n_sol):
                r1 = np.random.randint(0, pop.shape[0])
                chromosome = pop[r1, :]
                r2 = np.random.randint(0, pop.shape[1])
                chromosome[r2] += np.random.uniform(-step_size, step_size)
                if chromosome[r2] < eps:
                    chromosome[r2] = eps
                if chromosome[r2] > -eps:
                    chromosome[r2] = -eps

                offspring[i,:] = chromosome
            return offspring
        
        rate_crossover = 20         # number of chromosomes that we apply crossower to
        rate_mutation = 20          # number of chromosomes that we apply mutation to
        rate_local_search = 10      # number of chromosomes that we apply local_search to
        step_size = 0.02
        pop_size = 100

        pop = 2.0* np.random.random (size= (pop_size,70))
        
        for iter in range(G_max):
            offspring_from_crossover = crossover(pop, rate_crossover)
            offspring_from_mutation = mutation(pop, rate_mutation)
            offspring_from_local_search = local_search(pop, rate_local_search, step_size)
            
            # we append childrens Q (cross-overs, mutations, local search) to paraents P
            # having parents in the mix, i.e. allowing for parents to progress to next iteration - Elitism
            pop = np.append(pop, offspring_from_crossover, axis=0)
            pop = np.append(pop, offspring_from_mutation, axis=0)
            pop = np.append(pop, offspring_from_local_search, axis=0)
            # print(pop.shape)
            fitness_values = objective_function(pop)
            pop = selection(pop, fitness_values, pop_size)  # we arbitrary set desired pereto front size = pop_size
            print('iteration: {0}  p: [{1}]'.format(iter, pop[0]))

        init_x0_best = pop[0]
        x_temp = copy.deepcopy(init_x0_best)
        a_init, b_init =  np.split(x_temp.reshape(10,7),2)
        output = _f(a_init, b_init)
        print("x_temp=", x_temp)
        raise ValueError("Run to here {0}".format(output))








def mainO(args=None):
    rclpy.init(args=args)
    paraEstimator = TrajGeneration()
    Ff = 0.1
    sampling_rate = 100.0
    sampling_rate_inoptimization = 20.0

    theta1 = 0.0
    theta2 = -0.5233

    a,b,fc = paraEstimator.generate_opt_traj_Link(Ff = Ff,sampling_rate = sampling_rate_inoptimization, bias = [theta1, theta2, 0.0, 0.0, 0.0, 0.0, 0.0])
    print("a = {0} \n b = {1}".format(a,b))

    ret = paraEstimator.generateToCsv(a,b,Ff = Ff,sampling_rate=sampling_rate)

    if ret:
        print("Done! Congratulations! self-collision avoidance")
        
        eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))

        print("fc = ",eigenvalues)
        print("a = {0} \n b = {1}".format(a,b))
        conditional_num = np.sqrt(eigenvalues[0]/eigenvalues[-1])
        print("conditional_num_best = ",conditional_num)

    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    paraEstimator = TrajGeneration()
    Ff = 0.1
    sampling_rate = 100.0
    sampling_rate_inoptimization = 20.0

    # first Try
    # theta_range = [-1.0, -0.7, -0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5, 0.7, 1.0]

    # Refine
    theta_range = [-0.5233, -0.2, 0.0]
    conditional_num_best = 1000000000.0
    for theta1 in theta_range:
        for theta2 in theta_range:
            a,b,fc = paraEstimator.generate_opt_traj_Link(Ff = Ff,sampling_rate = sampling_rate_inoptimization,bias = [theta1, theta2, 0.0, 0.0, 0.0, 0.0, 0.0])
            print("a = {0} \n b = {1}".format(a,b))
            eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))
            print("fc = ",eigenvalues)

            conditional_num = np.sqrt(eigenvalues[0]/eigenvalues[-1])

            if conditional_num<conditional_num_best:
                conditional_num_best = conditional_num
                best_theta = [theta1, theta2]
                a_best = a
                b_best = b
    print("conditional_num_best = ",conditional_num_best)
    print("best_theta = ",best_theta)




    ret = paraEstimator.generateToCsv(a,b,Ff = Ff,sampling_rate=sampling_rate)

    if ret:
        print("Done! Congratulations! self-collision avoidance")
        
        eigenvalues, eigenvectors = np.linalg.eig(fc(a,b))

        print("fc = ",eigenvalues)
        print("a = ",a_best)
        print("b = ",b_best)

    rclpy.shutdown()


if __name__ == "__main__":
    mainO()