#!/usr/bin/env python3
"""
Simple FK Test - Input joint angles, output Cartesian coordinates.
Uses the same mounting configuration as ik_test.py
"""
import sys
import os
import numpy as np
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from ik_qp import QPIK, deg2rad, rad2deg

class FKTester:
    def __init__(self, robot_type="RM75B", install_angle=None, work_cs=None, tool_cs=None):
        self.ik_solver = QPIK(robot_type, dT=0.02)
        
        # Same defaults as ik_test.py
        if install_angle is None: install_angle = [0, 0, 0]
        if work_cs is None: work_cs = [0, 0, 0, 0, 0, 0]
        if tool_cs is None: tool_cs = [0, 0, 0, 0, 0, 0]
        
        self.ik_solver.set_install_angle(install_angle, 'deg')
        self.ik_solver.set_work_cs_params(work_cs)
        self.ik_solver.set_tool_cs_params(tool_cs)
        
        print(f"FK Config: Install={install_angle}, Work={work_cs}, Tool={tool_cs}")
    
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        assert(self.isRotationMatrix(R))
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])

    def rotationMatrixToQuaternion(self, R):
        """Convert rotation matrix to quaternion [w, x, y, z]"""
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1] - R[1,2]) * s
            y = (R[0,2] - R[2,0]) * s
            z = (R[1,0] - R[0,1]) * s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
        return np.array([w, x, y, z])

    def compute_fk(self, joints_rad):
        """
        Input: joint angles in radians [j1, j2, j3, j4, j5, j6, j7]
        Returns: dict with position, euler (rad), and quaternion
        """
        q_rad = np.array(joints_rad)
        
        rbt = self.ik_solver.robot
        T_mat = rbt.fkine(q_rad)
        
        # Extract position (meters)
        x, y, z = T_mat[0,3], T_mat[1,3], T_mat[2,3]
        
        # Extract rotation
        R_mat = np.array(T_mat[0:3, 0:3])
        euler = self.rotationMatrixToEulerAngles(R_mat)
        quat = self.rotationMatrixToQuaternion(R_mat)
        
        return {
            'position': [x, y, z],
            'euler_rad': euler.tolist(),
            'quaternion': quat.tolist()  # [w, x, y, z]
        }


if __name__ == "__main__":
    fk = FKTester()
    
    # Test joint angles (radians) - modify as needed
    test_joints = [0.03936, 2.06, -0.184, -0.0425, -1.463426, 0.05666, 1.7223]
    
    print(f"\nInput Joints (rad): {test_joints}")
    result = fk.compute_fk(test_joints)
    
    pos = result['position']
    euler = result['euler_rad']
    quat = result['quaternion']
    
    print(f"  Position (m): x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
    print(f"  Euler (rad): rx={euler[0]:.4f}, ry={euler[1]:.4f}, rz={euler[2]:.4f}")
    print(f"  Euler (deg): rx={np.degrees(euler[0]):.2f}, ry={np.degrees(euler[1]):.2f}, rz={np.degrees(euler[2]):.2f}")
    print(f"  Quaternion [w,x,y,z]: {np.round(quat, 4)}")
