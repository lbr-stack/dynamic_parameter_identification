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
from IDmodel import TD_2order, TD_list_filter,find_dyn_parm_deps, RNEA_function,DynamicLinearlization,getJointParametersfromURDF
from scipy import signal

Order = [0,1,2,3,4,5,6]


class Estimator():
    def __init__(self, node_name = "para_estimatior", dt_ = 5.0, N_ = 100, gravity_vec = [0.0, 0.0, -9.81]) -> None:

        self.dt_ = dt_
        # self.declare_parameter("model", "med7dock")
        self.model_ = "med7dock" #str(self.get_parameter("model").value)
        path = os.path.join(
            get_package_share_directory("med7_dock_description"),
            "urdf",
            # self.model_,
            f"{self.model_}.urdf.xacro",
        )
        self.N = N_

        # 1. Get the kinematic parameters of every joints
        self.robot = optas.RobotModel(
            xacro_filename=path,
            time_derivs=[1],  # i.e. joint velocity
        )


        Nb, xyzs, rpys, axes = getJointParametersfromURDF(self.robot)
        self.dynamics_ = RNEA_function(Nb,1,rpys,xyzs,axes,gravity_para = cs.DM(gravity_vec))
        self.Ymat, self.PIvector = DynamicLinearlization(self.dynamics_,Nb)


        urdf_string_ = xacro.process(path)
        robot = urdf.URDF.from_xml_string(urdf_string_)

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
    

    
    def ExtractFromMeasurmentCsv(self,path_pos):

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
        # filter = TD_2order(T=0.01)
        for id in range(len(pos_l)):
            if id == 0:
                vel_l.append([0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0])
            else:
                vel_l.append([(p-p_1)/dt for (p,p_1) in zip(pos_l[id],pos_l[id-1])])



        return pos_l,vel_l,tau_ext_l    
    
    def ExtractFromMeasurmentList(self,pos_list):

        dt = 0.01
        pos_l = []
        tau_ext_l = []
        # with open(path_pos) as csv_file:
        #     csv_reader = csv.DictReader(csv_file)
        for row in pos_list:
            # print("111 = {0}".format(row.values()))
            pl = row[0:7]
            tl = row[7:14]
            pos_l.append([float(x) for x in pl])
            tau_ext_l.append([float(x) for x in tl])

        vel_l =[]
        # filter = TD_2order(T=0.01)
        for id in range(len(pos_l)):
            if id == 0:
                vel_l.append([0.0, 0.0,0.0, 0.0,0.0, 0.0,0.0])
            else:
                vel_l.append([(p-p_1)/dt for (p,p_1) in zip(pos_l[id],pos_l[id-1])])



        return pos_l,vel_l,tau_ext_l    
       

 
    
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
    

    def timer_cb_regressor_physical_con(self, positions, velocities, efforts):
        
        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        # q_nps = []
        # qd_nps = []
        # qdd_nps = []
        taus = []
        Y_ = []
        Y_fri = []
        # init_para = np.random.uniform(0.0, 0.1, size=50)
        
        # filter_list = [TD_2order(T=0.01) for i in range(7)]
        # filter_vector = TD_list_filter(T=0.01)
        for k in range(0,len(positions),1):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            q_np = [positions[k][i] for i in Order]
            # print("velocities[k] = {0}".format(velocities[k]))
            qd_np = [velocities[k][i] for i in Order]
            tau_ext = [efforts[k][i] for i in Order]

            qdlast_np = [velocities[k-1][i] for i in Order]
            
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
            qdd_np = qdd_np.tolist()
    

            Y_temp = self.Ymat(q_np,
                               qd_np,
                               qdd_np) @Pb 
            fri_ = np.diag([float(np.sign(item)) for item in qd_np])
            fri_ = np.hstack((fri_,  np.diag(qd_np)))
            # fri_ = [[np.sign(v), v] for v in qd_np]
            
            Y_.append(Y_temp)
   
            taus.append(tau_ext)
            Y_fri.append(np.asarray(fri_))
            
            # print(qdd_np)

        
        Y_r = optas.vertcat(*Y_)

        taus1 = np.hstack(taus)
        Y_fri1 = np.vstack(Y_fri)

        pa_size = Y_r.shape[1]
 


        taus1 = taus1.T



   

        # Y = Y_r #cs.DM(np.hstack((Y_r, Y_fri1)))
        Y = cs.DM(np.hstack((Y_r, Y_fri1)))
        estimate_pam = np.linalg.inv(Y.T @ Y) @ Y.T @ taus1
 
        # print("self.masses_np",self.masses_np.shape)
        # print("self.masses_np",self.massesCenter_np.shape)

        _w1, _h1 =self.massesCenter_np.shape
        _w2, _h2 =self.Inertia_np.shape
        _w0 = len(self.masses_np)
        l = _w0 + _h1*_w1 + _w2 * _h2
        l1 = _w0 + _w1*_h1

        _estimate = cs.SX.sym('para', l)

        estimate_cs = K @ self.PIvector(_estimate[0:_w0],
                                        _estimate[_w0:l1].reshape((_w1,_h1)),
                                        _estimate[l1:l].reshape((_w2,_h2))
                                        )
        obj = cs.sumsqr(taus1 - Y @ estimate_cs)


        # lb = -3.0*np.array([1.0]*(pa_size))
        # ub = 3.0*np.array([1.0]*(pa_size))

        # print("self.masses_npv", self.masses_np.shape)
       
        # ref_pam = K @ self.PIvector(self.masses_np,self.massesCenter_np,self.Inertia_np).toarray().flatten()

        # print("ref_pam = ",ref_pam.shape)
        # print("lb = ",lb.shape)
        
        # lb[:pa_size] = -2.0*ref_pam
        # ub[:pa_size] = 2.0*ref_pam

        mu_mc = _estimate[_w0:l1]
        mass_norminal = self.masses_np
        mass_center_norminal = self.massesCenter_np.reshape(-1,_w1*_h1).flatten()
        intertia_norminal = self.Inertia_np.reshape(-1,_w2*_h2).flatten()
        
        Inertia = _estimate[l1:l].reshape((_w2,_h2))

        print("_w2, _h2 = {0}, {1}".format(_w2, _h2))
        list_of_intertia_norminal = [Inertia[:, i:i+3] for i in range(0, Inertia.shape[1], 3)]
        # raise ValueError("Run to here")


        # ineq_constr = [estimate_cs[i] >= lb[i] for i in range(pa_size)] + [estimate_cs[i] <= ub[i] for i in range(pa_size)]
        ineq_constr = []
        ineq_constr += [_estimate[i]> 0.0 for i in range(_w0)]
        ineq_constr += [_estimate[i] - mass_norminal[i]<= 0.2*cs.norm_2(mass_norminal[i]) for i in range(_w0)]
        ineq_constr += [_estimate[i] - mass_norminal[i]>= -0.2*cs.norm_2(mass_norminal[i]) for i in range(_w0)]

        ineq_constr += [_estimate[_w0+i] - mass_center_norminal[i]<= 0.5*cs.norm_2(mass_center_norminal[i]) for i in range(_w1*_h1)]
        ineq_constr += [_estimate[_w0+i] - mass_center_norminal[i]>= -0.5*cs.norm_2(mass_center_norminal[i]) for i in range(_w1*_h1)]

        ineq_constr += [_estimate[_w0+_w1*_h1+i] - intertia_norminal[i]<= 0.5*cs.norm_2(intertia_norminal[i]) for i in range(_w2*_h2)]
        ineq_constr += [_estimate[_w0+_w1*_h1+i] - intertia_norminal[i]>= -0.5*cs.norm_2(intertia_norminal[i]) for i in range(_w2*_h2)]

        # print("list_of_intertia_norminal = {0}".format(list_of_intertia_norminal[0]))
        ineq_constr += [I[0,0] <=I[1,1] +I[2,2] for I in list_of_intertia_norminal]
        ineq_constr += [I[1,1] <=I[0,0] +I[2,2] for I in list_of_intertia_norminal]
        ineq_constr += [I[2,2] <=I[1,1] +I[0,0] for I in list_of_intertia_norminal]

        ineq_constr += [100.0*cs.mmin(cs.vertcat(I[1,1], I[0,0], I[2,2]))  >=cs.mmax(cs.vertcat(I[1,1], I[0,0], I[2,2])) for I in list_of_intertia_norminal]

        ineq_constr += [cs.trace(I)>0.0 for I in list_of_intertia_norminal]
        ineq_constr += [cs.det(I)>0.0 for I in list_of_intertia_norminal]
        ineq_constr += [cs.det(I)>0.0 for I in list_of_intertia_norminal]
        ineq_constr += [I[0,1]>= I[1,0] for I in list_of_intertia_norminal]
        ineq_constr += [I[0,1]<= I[1,0] for I in list_of_intertia_norminal]

        ineq_constr += [I[0,2]>= I[2,0] for I in list_of_intertia_norminal]
        ineq_constr += [I[0,2]<= I[2,0] for I in list_of_intertia_norminal]

        ineq_constr += [I[2,1]>= I[1,2] for I in list_of_intertia_norminal]
        ineq_constr += [I[2,1]<= I[1,2] for I in list_of_intertia_norminal]


        ineq_constr += [1e-4<= I[0,0] for I in list_of_intertia_norminal]
        ineq_constr += [1e-4<= I[1,1] for I in list_of_intertia_norminal]
        ineq_constr += [1e-4<= I[2,2] for I in list_of_intertia_norminal]


        ineq_constr += [1e-4<= I[2,2] for I in list_of_intertia_norminal]


        ineq_constr += [cs.mmax(cs.vertcat(
                                            cs.norm_2(I[1,0]), 
                                            cs.norm_2(I[0,2]), 
                                            cs.norm_2(I[1,2])
                                           ))<= 0.1*cs.norm_2(cs.mmin(cs.vertcat(I[1,1], I[0,0], I[2,2]))) for I in list_of_intertia_norminal]
        
        ineq_constr += [3.0 * list_of_intertia_norminal[j][2,2]<= cs.mmin(cs.vertcat(list_of_intertia_norminal[j][0,0], list_of_intertia_norminal[j][1,1])) for j in [0, 2, 3]]
        ineq_constr += [3.0 * list_of_intertia_norminal[j][1,1]<= cs.mmin(cs.vertcat(list_of_intertia_norminal[j][0,0], list_of_intertia_norminal[j][2,2])) for j in [1, 3]]
        

        # ineq_constr += [cs.norm_2(_estimate[_w0+i] - mass_center_norminal[i])> 0.1*cs.norm_2(mass_center_norminal[i]) for i in range(_w1*_h1)]
        # ineq_constr += [_estimate[i]> 0.0 for i in range(_w2*_h2)]

        problem = {'x': _estimate, 'f': obj, 'g': cs.vertcat(*ineq_constr)}
        # solver = cs.qpsol('solver', 'qpoases', problem)
        # solver = cs.nlpsol('S', 'ipopt', problem,{'ipopt':{'max_iter':3000000 }, 'verbose':True})
        solver = cs.nlpsol('S', 'ipopt', problem,
                      {'ipopt':{'max_iter':1 }, 
                       'verbose':False,
                       "ipopt.hessian_approximation":"limited-memory"
                       })
        
        print("solver = {0}".format(solver))
        # sol = S(x0 = init_x0,lbg = lbg, ubg = ubg)
        init_x0 = mass_norminal.tolist()+mass_center_norminal.tolist()+intertia_norminal.tolist()
        sol = solver(x0 = np.asarray(init_x0))

        print("sol = {0}".format(sol['x']))

        return sol['x'],estimate_pam
    

    def timer_cb_regressor(self, positions, velocities, efforts):
        
        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        # q_nps = []
        # qd_nps = []
        # qdd_nps = []
        taus = []
        Y_ = []
        Y_fri = []
        # init_para = np.random.uniform(0.0, 0.1, size=50)
        
        # filter_list = [TD_2order(T=0.01) for i in range(7)]
        # filter_vector = TD_list_filter(T=0.01)
        for k in range(0,len(positions),1):
            # print("q_np = {0}".format(q_np))
            # q_np = np.random.uniform(-1.5, 1.5, size=7)
            q_np = [positions[k][i] for i in Order]
            # print("velocities[k] = {0}".format(velocities[k]))
            qd_np = [velocities[k][i] for i in Order]
            tau_ext = [efforts[k][i] for i in Order]

            qdlast_np = [velocities[k-1][i] for i in Order]
            
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
            qdd_np = qdd_np.tolist()
    

            Y_temp = self.Ymat(q_np,
                               qd_np,
                               qdd_np) @Pb 
            fri_ = np.diag([float(np.sign(item)) for item in qd_np])
            fri_ = np.hstack((fri_,  np.diag(qd_np)))
            # fri_ = [[np.sign(v), v] for v in qd_np]
            
            Y_.append(Y_temp)
   
            taus.append(tau_ext)
            Y_fri.append(np.asarray(fri_))
            
            # print(qdd_np)

        
        Y_r = optas.vertcat(*Y_)

        taus1 = np.hstack(taus)
        Y_fri1 = np.vstack(Y_fri)
        print("Y_fri1 = {0}".format(Y_fri1))
        print("Y_fri1 = {0}".format(Y_fri1.shape))
        print("Y_r = {0}".format(Y_r.shape))
        print("Pb = {0}".format(Pb.shape))
        pa_size = Y_r.shape[1]
 


        taus1 = taus1.T


        # estimate_pam = np.linalg.inv(Y_r.T @ Y_r) @ Y_r.T @ taus1

   

        Y = cs.DM(np.hstack((Y_r, Y_fri1)))
        estimate_pam = np.linalg.inv(Y.T @ Y) @ Y.T @ taus1
 
        

        estimate_cs = cs.SX.sym('para', pa_size+14)
        obj = cs.sumsqr(taus1 - Y @ estimate_cs)


        lb = -3.0*np.array([1.0]*(pa_size+14))
        ub = 3.0*np.array([1.0]*(pa_size+14))

        print("self.masses_npv", self.masses_np.shape)
       
        ref_pam = K @ self.PIvector(self.masses_np,self.massesCenter_np,self.Inertia_np).toarray().flatten()

        print("ref_pam = ",ref_pam.shape)
        print("lb = ",lb.shape)
        
        lb[:pa_size] = -2.0*ref_pam
        ub[:pa_size] = 2.0*ref_pam



        ineq_constr = [estimate_cs[i] >= lb[i] for i in range(pa_size)] + [estimate_cs[i] <= ub[i] for i in range(pa_size)]

        problem = {'x': estimate_cs, 'f': obj, 'g': cs.vertcat(*ineq_constr)}
        # solver = cs.qpsol('solver', 'qpoases', problem)
        solver = cs.nlpsol('S', 'ipopt', problem,{'ipopt':{'max_iter':3000000 }, 'verbose':True})
        print("solver = {0}".format(solver))
        sol = solver()

        print("sol = {0}".format(sol['x']))

        return sol['x'],estimate_pam
    
    def testWithEstimatedParaCon(self, positions, velocities, efforts, para)->None:

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        tau_ests = []
        es = []

        filter_list = [TD_2order(T=0.01) for i in range(7)]
        _w1, _h1 =self.massesCenter_np.shape
        _w2, _h2 =self.Inertia_np.shape
        _w0 = len(self.masses_np)
        l = _w0 + _h1*_w1 + _w2 * _h2
        l1 = _w0 + _w1*_h1

        # _estimate = cs.SX.sym('para', l)

        estimate_cs = K @ self.PIvector(para[0:_w0],
                                        para[_w0:l1].reshape((_w1,_h1)),
                                        para[l1:l].reshape((_w2,_h2)))
        for k in range(1,len(positions),1):


            q_np = [positions[k][i] for i in Order]
            qd_np = [velocities[k][i] for i in Order]
            tau_ext = [efforts[k][i] for i in Order]

            qdlast_np = [velocities[k-1][i] for i in Order]
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01#(velocities[k][0]-velocities[k-1][0])

            qdd_np = [f(qd_np[id])[1] for id,f in enumerate(filter_list)]

            pa_size = Pb.shape[1]

            tau_est_model = (self.Ymat(q_np,qd_np,qdd_np) @Pb@  estimate_cs )
            e= tau_est_model - tau_ext 
            print("error1 = {0}".format(e))
            print("tau_ext = {0}".format(tau_ext))

            tau_ests.append(tau_est_model.toarray().flatten().tolist())
            es.append(e.toarray().flatten().tolist())

        return tau_ests, es
    
    def testWithEstimatedPara(self, positions, velocities, efforts, para)->None:

        Pb, Pd, Kd =find_dyn_parm_deps(7,80,self.Ymat)
        K = Pb.T +Kd @Pd.T

        tau_ests = []
        es = []

        filter_list = [TD_2order(T=0.01) for i in range(7)]
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
            qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01#(velocities[k][0]-velocities[k-1][0])
            # qdd_np = qdd_np.tolist()
            # qdd_np = (np.array(qd_np)-np.array(qdlast_np))/0.01
            # qdd_np = qdd_np.tolist()
            qdd_np = [f(qd_np[id])[1] for id,f in enumerate(filter_list)]

            # tau_ext = self.robot.rnea(q_np,qd_np,qdd_np)
            # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @ (solution[f"{self.pam_name}/y"] -  K @real_pam)
            # print("error = {0}".format(e))

            # e=self.Ymat(q_np,qd_np,qdd_np)@Pb @  para - tau_ext 
            pa_size = Pb.shape[1]
            # tau_est_model = (self.Ymat(q_np,qd_np,qdd_np) @Pb@  para[:pa_size] + 
            #     np.diag(np.sign(qd_np)) @ para[pa_size:pa_size+7]+ 
            #     np.diag(qd_np) @ para[pa_size+7:])
            tau_est_model = (self.Ymat(q_np,qd_np,qdd_np) @Pb@  para[:pa_size] )
            e= tau_est_model - tau_ext 
            print("error1 = {0}".format(e))
            print("tau_ext = {0}".format(tau_ext))

            tau_ests.append(tau_est_model.toarray().flatten().tolist())
            es.append(e.toarray().flatten().tolist())

        return tau_ests, es


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
    
        

def traj_filter(states):
    cols = []
    l=len(states[0])

    fs = 100
    cutoff_freq = 2  # 截止频率为10 Hz
    b, a = signal.butter(4, cutoff_freq / (fs / 2), 'low')
    filtered_signal = []
    states_filtered = []


    for i in range(l):
        # print("i = ",i)
        cols.append(
            [float(state[i]) for state in states]
        )

        filtered_signal.append( signal.filtfilt(b, a, cols[i]))


    for j in range(len(filtered_signal[0])):
        states_filtered.append([
            filtered_signal[i][j] for i in range(l)
        ])

    return states_filtered



def compare_traj(states1, states2):
    col1s , col2s = [], []
    l=len(states1[0])

    fig, axs = plt.subplots(7, 1, figsize=(8,10))

    for i in range(l):
        print("states = {0}".format(states2[i]))
        col1s.append(
            [float(state[i]) for state in states1]
        )
        col2s.append(
            [float(state[i]) for state in states2]
        )
        axs[i].plot(col1s[i])
        axs[i].plot(col2s[i])

    plt.subplots_adjust(hspace=0.5)
    plt.show()






def main(args=None):
    rclpy.init(args=args)
    paraEstimator = Estimator()

    path_pos = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "robot_data copy 2.csv",
        )

    positions, velocities, efforts = paraEstimator.ExtractFromMeasurmentCsv(path_pos)
    velocities=traj_filter(velocities)
    efforts_f=traj_filter(efforts)



    estimate_pam,ref_pam = paraEstimator.timer_cb_regressor_physical_con(positions, velocities, efforts_f)
    print("estimate_pam = {0}".format(estimate_pam))
    tau_exts, es =paraEstimator.testWithEstimatedParaCon(positions, velocities, efforts_f,estimate_pam)
    paraEstimator.saveEstimatedPara(estimate_pam)
    compare_traj(tau_exts, efforts_f)


    path_pos_2 = os.path.join(
            get_package_share_directory("gravity_compensation"),
            "test",
            "measurements_0dgr.csv",
        )

    positions_, velocities_, efforts_ = paraEstimator.ExtractFromMeasurmentCsv(path_pos_2)
    velocities_=traj_filter(velocities_)
    efforts_f_=traj_filter(efforts_)
    tau_exts_, es =paraEstimator.testWithEstimatedParaCon(positions_, velocities_, efforts_f_,estimate_pam)
    compare_traj(tau_exts_, efforts_f_)

    rclpy.shutdown()



if __name__ == "__main__":
    main()