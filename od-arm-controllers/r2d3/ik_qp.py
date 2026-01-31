#! -*-conding=: UTF-8 -*-
#
################################################################################
#                                                                              #
# Author   : Daryl/Ray/Leon                                                    #
# Date     : 2024/09/26                                                        #
# Copyright: Copyright (c) 2018-2024 RealMan Co., Ltd.. All rights reserved.   #
#                                                                              #
################################################################################
import qpSWIFT
import numpy as np
from ik_rbtdef import *
from ik_rbtutils import *
from Robotic_Arm.rm_robot_interface import *
import time
##########################################################################################
#####   Be sure to test in simulation mode before debugging, and only test on the real robot arm if there are no problems!!!   #####
##########################################################################################
#!!!!!!!!    After reading the following interface instructions, please move to Rm65_demo and carefully read every line of comments !!!!!!!!!!!!!!#
class QPIK():
    '''
        f(x) = 0.5*x'*P*x + c'*x
        subject to G*x <= h
               
                 ||
               \ || /
                 \/
        
        f(dq) = ||Jdq - dX||^2
        subject to G*dq <= h

        where,
            P = J^T * W^T * W * J
            c = -2 * J^T * W^T * W * dX
            G = [I, -I, I, -I]^T
            h = [dq_w.*dq_max, -dq_w.*dq_max, q_max - q_ref, q_ref - q_min]^T
    '''
    def __init__(self, type, dT):
        self.dT = dT
        self.robot = Robot(type)

        self.q_max, self.q_min = self.robot.get_qlim()
        self.dq_max = self.robot.get_dq_max().transpose()

        self.W = np.eye(6)
        self.dq_weight = np.ones(self.robot.dof)

        # Limit wrist velocity.
        if self.robot.dof == 7:
            self.dq_weight[4] = 0.1
        else:
            self.dq_weight[3] = 0.1
            
        self.damping = 0.0 # Default to 0

    def set_7dof_q3_max_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_max[2] = angle * deg2rad
        else:
            self.q_max[2] = angle

    def set_7dof_q3_min_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_min[2] = angle * deg2rad
        else:
            self.q_min[2] = angle

    def set_7dof_elbow_max_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_max[3] = angle * deg2rad
        else:
            self.q_max[3] = angle

    def set_7dof_elbow_min_angle(self,angle,unit = 'rad'):
        if unit == 'deg':
            self.q_min[3] = angle * deg2rad
        else:
            self.q_min[3] = angle
    
    def set_6dof_elbow_max_angle(self, angle, unit='rad'):
        if unit == 'deg':
            self.q_max[2] = angle * deg2rad
        else:
            self.q_max[2] = angle

    def set_6dof_elbow_min_angle(self, angle, unit='rad'):
        if unit == 'deg':
            self.q_min[2] = angle * deg2rad
        else:
            self.q_min[2] = angle
    
    def set_joint_limit_max(self, angle, unit = 'rad'):
        if len(angle)!= self.robot.dof:
            raise Exception(f"[ERROR] joint_limit_max size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.q_max[i] = angle[i] * deg2rad
            else:
                self.q_max[i] = angle[i]

    def set_joint_limit_min(self,angle, unit = 'rad'):
        if len(angle)!= self.robot.dof:
            raise Exception(f"[ERROR] joint_limit_min size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.q_min[i] = angle[i] * deg2rad
            else:
                self.q_min[i] = angle[i]

    def set_install_angle(self, angle, unit='rad'):
        self.robot.set_install_angle(angle, unit)

    def set_work_cs_params(self, pose):
        self.robot.set_work_cs_params(pose)
    
    def set_tool_cs_params(self, pose):
        self.robot.set_tool_cs_params(pose)

    def set_error_weight(self, weight):
        if len(weight)!= 6:
            raise Exception(f"[ERROR] weight size should be {6}.")
        for i in range(6):
            if weight[i] > 1.0:
                weight[i] = 1.0
            elif weight[i] < 0.0:
                weight[i] = 0.0
            self.W[i,i] = weight[i]
    
    def set_joint_velocity_limit(self, dq_max, unit='rad'):
        if len(dq_max)!= self.robot.dof:
            raise Exception(f"[ERROR] dq_max size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if unit == 'deg':
                self.dq_max[i] = dq_max[i] * deg2rad
            else:
                self.dq_max[i] = dq_max[i]
    
    def set_dq_max_weight(self, weight):
        if len(weight)!= self.robot.dof:
            raise Exception(f"[ERROR] weight size should be {self.robot.dof}.")
        for i in range(self.robot.dof):
            if weight[i] > 1.0:
                weight[i] = 1.0
            elif weight[i] < 0.0:
                weight[i] = 0.0
            self.dq_weight[i] = weight[i]    

    def fkine(self, q):
        return self.robot.fkine(q)    
    
    def __Limt(self,q_ref):
        self.q_ref = np.array(q_ref)

        G1 =      np.eye(self.robot.dof) # 关节速度约束
        G2 = -1 * np.eye(self.robot.dof)
        G3 =      np.eye(self.robot.dof) # 关节限位约束
        G4 = -1 * np.eye(self.robot.dof)

        self.G = np.concatenate([G1,G2,G3,G4])

        self.delta_q_max = np.zeros(self.robot.dof)
        
        for i in range(self.robot.dof):
            self.delta_q_max[i] = self.dq_max[i] * self.dq_weight[i] * self.dT
        
        h1 = self.delta_q_max
        h2 = self.delta_q_max
        h3 = self.q_max - self.q_ref
        h4 = self.q_ref - self.q_min

        self.h = np.concatenate([h1,h2,h3,h4])

    def set_damping(self, damping):
        self.damping = damping

    def sovler(self, q_ref, Td, max_iter = 150):

        self.q_sovle = np.zeros(self.robot.dof)
      
        self.Jaco = self.robot.jacob_Jw(q_ref) 
        self.Jaco = np.array(self.Jaco)

        Tc = self.fkine(q_ref)

        self.DX = angle_axis_diff(Tc,Td)
        self.__Limt(q_ref)
 
        self.c = -2*np.dot(self.Jaco.transpose() @ self.W.transpose() @ self.W, self.DX)
        # Add Damping term to P: (J^T W^T W J + lambda * I)
        # P defines the quadratic cost 0.5 * x^T P x.
        # To add regularization term 0.5 * lambda * ||x||^2, we add lambda * I to P.
        self.P =  2* self.Jaco.transpose() @ self.W.transpose() @ self.W @ self.Jaco
        if self.damping > 0:
            self.P += self.damping * np.eye(self.robot.dof)

        opts = {"MAXITER": max_iter, "VERBOSE": 0,"OUTPUT": 1}
        self.k = qpSWIFT.run(self.c, self.h, self.P, self.G, np.zeros([1,self.robot.dof]), np.zeros(1), opts)

        is_nan = False
        for j in range(self.robot.dof):
            is_nan = is_nan or np.isnan(self.k['sol'][j])

        is_success = True
        for j in range(self.robot.dof):
            if np.abs( self.k['sol'][j]) > 1.0 * self.q_max[j]:
                is_success = False

        if self.k['basicInfo']['ExitFlag'] == 1 or self.k['basicInfo']['ExitFlag'] == 3 or is_nan:
            for j in range(self.robot.dof):
                self.q_sovle[j] = q_ref[j]
            return self.q_sovle
    
        for j in range(self.robot.dof):
            self.q_sovle[j] = q_ref[j] + self.k['sol'][j]
        return self.q_sovle





