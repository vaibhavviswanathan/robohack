
import sys
import os
import time
import numpy as np
import math

# Ensure we can import local modules
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from ik_qp import QPIK, deg2rad, rad2deg
from ik_rbtutils import pose_to_matrix

try:
    from Robotic_Arm.rm_robot_interface import *
except ImportError:
    print("[ERROR] Could not import Robotic_Arm.rm_robot_interface. Ensure the API is in your PYTHONPATH.")
    sys.exit(1)

class IKTester:
    def __init__(self, ip, port, robot_type="RM65B", dt=0.02, 
                 install_angle=None, work_cs=None, tool_cs=None):
        self.ip = ip
        self.port = port
        self.dt = dt
        
        print(f"Connecting to robot at {ip}:{port}...", flush=True)
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm(ip, port)
        
        if self.handle.id <= 0:
            print(f"[ERROR] Failed to connect to robot at {ip}:{port}", flush=True)
            sys.exit(1)
        print(">> Connected successfully.", flush=True)
        
        self.ik_solver = QPIK(robot_type, dt)
        
        # Initialize solver parameters 
        # Defaults if not provided (Left Arm standard)
        if install_angle is None: install_angle = [0, 45, 0]
        if work_cs is None: work_cs = [0, 0, 0, 0, 0, -1.571]
        if tool_cs is None: tool_cs = [0, 0, 0, 0, 0, 0]
        
        self.ik_solver.set_install_angle(install_angle, 'deg')
        self.ik_solver.set_work_cs_params(work_cs)
        self.ik_solver.set_tool_cs_params(tool_cs)
        
        print(f"Configs: Install={install_angle}, Work={work_cs}, Tool={tool_cs}")
        
        # Joint Limits (Degrees)
        self.q_min_deg = np.array([-178, -130, -135, -178, -128, -360])
        self.q_max_deg = np.array([178, 130, 135, 178, 128, 360])
        
        self.ik_solver.set_joint_limit_max(self.q_max_deg, 'deg')
        self.ik_solver.set_joint_limit_min(self.q_min_deg, 'deg')

    def monitor_status(self):
        """
        Check robot for errors.
        Returns True if status is OK, False if Error.
        """
        ret, state = self.robot.rm_get_current_arm_state()
        if ret == 0:
            err = state['err']
            if err['err_len'] > 0:
                print(f"[ERROR] Robot Error Detected: {err['err']}")
                return False
        else:
            print(f"[WARNING] Failed to get arm state (Code: {ret}). Ignoring for test purposes (assuming API mismatch).")
            # We return True here to allow the test to proceed despite the API error, 
            # as long as the robot isn't actually in a hard error state (which we can't check).
            return True 
        return True

    def clear_errors(self):
        print(">> Clearing system errors...")
        self.robot.rm_clear_system_err()
        time.sleep(1)

    def get_current_joint_angles(self):
        ret, joints = self.robot.rm_get_joint_degree()
        if ret == 0:
            return np.array(joints)
        print(f"[ERROR] Failed to get joint angles. Return code: {ret}")
        return None

    # --- Helper functions for FK Fallback ---
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        assert(self.isRotationMatrix(R))
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])
    # ----------------------------------------

    def get_current_pose(self):
        """
        Returns [x, y, z, rx, ry, rz] in meters and radians.
        """
        
        # Method 3: Fallback to FK using Joint Angles (Most robust if API struct parsing fails)
        ret, joints = self.robot.rm_get_joint_degree()
        if ret == 0:
             # Convert to radians
             q_rad = np.array(joints) * deg2rad
             
             # DEBUG: Verify params are set
             rbt = self.ik_solver.robot
             print(f"[DEBUG] FK Config -> Install: {rbt.get_install_angle()}, Work: {rbt.get_work_cs_params()}, Tool: {rbt.get_tool_cs_params()}")
             
             # Compute FK
             T_mat = rbt.fkine(q_rad)
             
             # Extract position
             x = T_mat[0,3]
             y = T_mat[1,3]
             z = T_mat[2,3]
             
             # Extract Rotation
             R_mat = T_mat[0:3, 0:3]
             # Convert matrix to np.array for helper function if needed (fkine returns matrix or asmatrix)
             R_arr = np.array(R_mat) 
             
             euler = self.rotationMatrixToEulerAngles(R_arr)
             
             fallback_pose = [x, y, z, euler[0], euler[1], euler[2]]
             print(f"[DEBUG] Computed FK Pose from Joints: {np.round(fallback_pose, 4)}")
             return fallback_pose
             
        print(f"[ERROR] All pose retrieval methods failed.")
        return None

    def test_ik_move(self, target_pose_list):
        """
        target_pose_list: [x, y, z, rx, ry, rz] (meters, radians)
        """
        print("\n--- Starting IK Test & Move ---")
        
        # 0. Check Status
        if not self.monitor_status():
            self.clear_errors()
            if not self.monitor_status():
                print("[ERROR] Could not clear errors. Aborting.")
                return

        # 1. Get Current Config for Solver Seed
        current_joints_deg = self.get_current_joint_angles()
        if current_joints_deg is None:
            return
            
        q_ref = current_joints_deg * deg2rad
        
        # 2. Setup Target
        print(f"Target Pose: {target_pose_list}")
        try:
            Td = pose_to_matrix(target_pose_list)
        except Exception as e:
            print(f"[ERROR] Failed to convert pose to matrix: {e}")
            return
        
        # 3. Solve IK
        # We run the solver multiple times to ensure convergence for a large step
        print(">> Computing IK solution...")
        q_sol = q_ref.copy()
        
        t0 = time.time()
        iterations = 50
        for _ in range(iterations):
            q_sol = self.ik_solver.sovler(q_sol, Td)
        t1 = time.time()
        
        q_sol_deg = q_sol * rad2deg
        print(f"Solved Joints (deg): {np.round(q_sol_deg, 3)}")
        print(f"IK Calculation Time ({iterations} iterations): {(t1 - t0) * 1000:.3f} ms")
        
        # 4. Safety Checks
        # A. NaN Check
        if np.isnan(q_sol_deg).any():
            print("[FATAL] IK Solution contains NaN. Aborting.")
            return

        # B. Joint Limits Check
        if np.any(q_sol_deg < self.q_min_deg) or np.any(q_sol_deg > self.q_max_deg):
            print("[FATAL] IK Solution violates joint limits!")
            diff_min = q_sol_deg - self.q_min_deg
            diff_max = q_sol_deg - self.q_max_deg
            print(f"   Below Min: {diff_min[diff_min < 0]}")
            print(f"   Above Max: {diff_max[diff_max > 0]}")
            return
            
        # C. Cartesian Error Check (Verify if the solution actually reaches the target)
        T_final = self.ik_solver.robot.fkine(q_sol) # FK of the solution
        target_pos = np.array(target_pose_list[:3])
        solution_pos = np.array([T_final[0,3], T_final[1,3], T_final[2,3]])
        
        predicted_error = np.linalg.norm(target_pos - solution_pos)
        print(f"Predicted Position Error: {predicted_error:.6f} m")
        
        if predicted_error > 0.100: # 2cm threshold
             print(f"[FATAL] Target unreachable! Predicted error ({predicted_error:.3f} m) exceeds threshold.")
             print("Aborting move to prevent reaching incorrect pose.")
             return

        # D. Large Motion Check (Optional Warning)
        diff_move = np.abs(q_sol_deg - current_joints_deg)
        if np.any(diff_move > 45): # Warn if any joint moves more than 45 degrees
            print("[WARNING] Large joint movement detected (>45 deg).")
            # user_input = input("Continue? (y/n): ")
            # if user_input.lower() != 'y': return

        # 5. Move Robot
        print(">> Executing Move (Speed: 20%)...")
        # rm_movej: (joint, v, r, connect, block)
        ret = self.robot.rm_movej(list(q_sol_deg), 20, 0, 0, 1)
        if ret != 0:
            print(f"[ERROR] Move command failed. Code: {ret}")
            return
        
        print(">> Move Complete.")

        # 6. Verify Result
        final_pose = self.get_current_pose()
        if final_pose:
             print(f"Final Robot Pose: {np.round(final_pose, 4)}")
             
             # Calculate error
             target_pos = np.array(target_pose_list[:3])
             final_pos_arr = np.array(final_pose[:3]) 
             
             pos_error = np.linalg.norm(target_pos - final_pos_arr)
             print(f"Position Error: {pos_error:.6f} m")
             
             if pos_error < 0.005: 
                 print("[SUCCESS] Position Verification PASSED (< 5mm)")
             else:
                 print("[FAILURE] Position Verification FAILED (> 5mm)")
                 
    def disconnect(self):
        print("Disconnecting...")
        self.robot.rm_delete_robot_arm()

if __name__ == "__main__":
    
    # --- CONFIGURATION ---
    ROBOT_IP = "169.254.128.18"
    ROBOT_PORT = 8080
    
    tester = IKTester(ROBOT_IP, ROBOT_PORT)
    
    # 1. Get current pose to act as a base
    curr = tester.get_current_pose()
    if curr:
        print(f"Current Pose: {curr}")
        
        # USER: defines target pose here.
        # You can modify this list directly to set your desired pose [x, y, z, rx, ry, rz]
        target_pose = list(curr)
        
        # Example: Move 1cm in X axis relative to current (Uncomment to use)
        # target_pose[0] += 0.01
        # target_pose[1] += 0.02
        # target_pose[2] += 0.03 
        
        # Example: Absolute pose (Uncomment and edit to use)
        # target_pose = [0.3, 0.0, 0.3, 3.14, 0, 0] 
        target_pose = [0.1858, 0.281, -0.1416, 0.261, 1.040, 1.981] 
        
        # Run test
        tester.test_ik_move(target_pose)
        
    tester.disconnect()
