import sys
import os
import time
import numpy as np
import yaml
from threading import Thread

# Ensure paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
new_api_path = os.path.abspath(os.path.join(current_dir, "../RM_API2-1.0.6/Python"))
if os.path.exists(new_api_path):
    sys.path.insert(0, new_api_path)

from ik_qp import QPIK, deg2rad, rad2deg
from ik_rbtutils import pose_to_matrix
from Robotic_Arm.rm_robot_interface import *

# Global Controllers
LEFT_CONTROLLER = None
RIGHT_CONTROLLER = None
_FIRST_ARM_CREATED = False  # Track if first arm initialized thread mode

class VLAArmController:
    def __init__(self, arm_name):
        global _FIRST_ARM_CREATED
        
        self.arm_name = arm_name
        self.config = self._load_config(arm_name)
        if not self.config:
            raise ValueError(f"Could not load config for {arm_name}")
            
        print(f"[{arm_name.upper()}] Connecting to {self.config['ip']}...")
        
        # Per RealMan docs: First arm initializes thread mode, subsequent use RoboticArm()
        if not _FIRST_ARM_CREATED:
            self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            _FIRST_ARM_CREATED = True
        else:
            self.robot = RoboticArm()  # No mode for subsequent arms
        
        self.handle = self.robot.rm_create_robot_arm(self.config['ip'], self.config['port'])
        
        if self.handle.id <= 0:
            raise ConnectionError(f"Failed to connect to {arm_name}")
        
        print(f"[{arm_name.upper()}] Connected (Handle ID: {self.handle.id})")
            
        # Initialize IK
        self.ik_solver = QPIK("RM65B", 0.01) # 50Hz = 0.02s
        if 'install_angle' in self.config:
            self.ik_solver.set_install_angle(self.config['install_angle'], 'deg')
        if 'work_cs' in self.config:
            self.ik_solver.set_work_cs_params(self.config['work_cs'])
        if 'tool_cs' in self.config:
            self.ik_solver.set_tool_cs_params(self.config['tool_cs'])
        
        # Joint Limits (Degrees) - same as ik_test.py
        self.q_min_deg = np.array([-178, -130, -135, -178, -128, -360])
        self.q_max_deg = np.array([178, 130, 135, 178, 128, 360])
        
        self.ik_solver.set_joint_limit_max(self.q_max_deg, 'deg')
        self.ik_solver.set_joint_limit_min(self.q_min_deg, 'deg')
            
        # State
        self.q_ref = None
        self.gripper_state = 0.0 # 0=Open, 1=Closed
        self.last_gripper_cmd = -1.0 # Force update first time
        
        # Get Initial State
        self._sync_state()
        
    def _load_config(self, arm_name):
        config_path = os.path.join(current_dir, 'robot_config.yaml')
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get(arm_name)

    def _sync_state(self):
        ret, joints = self.robot.rm_get_joint_degree()
        if ret == 0:
            self.q_ref = np.array(joints) * deg2rad
        else:
            print(f"[{self.arm_name}] WARN: Failed to get initial joints")
    
    def get_current_pose(self):
        """
        Get the current Cartesian pose using FK (matches ik_test.py approach).
        Returns: [x, y, z, rx, ry, rz] or None
        """
        # Retry a few times in case of timing issues after connection
        for attempt in range(3):
            ret, joints = self.robot.rm_get_joint_degree()
            if ret == 0:
                break
            time.sleep(0.1)
        
        if ret != 0:
            print(f"[{self.arm_name}] ERROR: Could not get joints for FK (ret={ret})")
            return None
        
        q_rad = np.array(joints) * deg2rad
        
        # Compute FK
        rbt = self.ik_solver.robot
        T_mat = rbt.fkine(q_rad)
        
        # Extract position
        x = float(T_mat[0, 3])
        y = float(T_mat[1, 3])
        z = float(T_mat[2, 3])
        
        # Extract Rotation using ik_test.py approach
        R_mat = np.array(T_mat[0:3, 0:3])
        euler = self._rotation_matrix_to_euler(R_mat)
        
        return [x, y, z, euler[0], euler[1], euler[2]]
    
    def _rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (from ik_test.py)."""
        import math
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def step(self, target_pose, gripper):
        """
        Execute one control step with full safety checks.
        target_pose: [x, y, z, rx, ry, rz] (m, rad)
        gripper: 0.0 (Open) to 1.0 (Closed)
        Returns: dict with status and telemetry
        """
        status = "OK"
        
        # 1. Gripper Logic - MOVED TO END
        # cmd_grip = 1.0 if gripper > 0.5 else 0.0
        # ...

        # 2. Check we have a seed state
        if self.q_ref is None:
            self._sync_state()
            if self.q_ref is None:
                return {"status": "ERR_NO_STATE", "arm": self.arm_name}

        # 3. Solve IK
        Td = pose_to_matrix(target_pose)
        q_sol = self.q_ref.copy()
        
        try:
            for _ in range(15):  # Iterative solve for convergence
                q_sol = self.ik_solver.sovler(q_sol, Td)
        except Exception as e:
            return {"status": f"IK_ERR: {e}", "arm": self.arm_name}

        # 4. SAFETY CHECKS (from ik_test.py)
        q_sol_deg = q_sol * rad2deg
        cap_reason = None
        cap_details = {}
        
        # A. NaN Check
        if np.isnan(q_sol_deg).any():
            nan_joints = np.where(np.isnan(q_sol_deg))[0].tolist()
            return {
                "status": "ERR_NAN", 
                "arm": self.arm_name, 
                "target_pose": target_pose,
                "cap_reason": f"NaN in joints {nan_joints}",
            }
        
        # B. Joint Limits Check
        joints_below = np.where(q_sol_deg < self.q_min_deg)[0]
        joints_above = np.where(q_sol_deg > self.q_max_deg)[0]
        
        if len(joints_below) > 0 or len(joints_above) > 0:
            status = "ERR_JOINT_LIMIT"
            violations = []
            for j in joints_below:
                violations.append(f"J{j+1}={q_sol_deg[j]:.1f}° < min={self.q_min_deg[j]:.0f}°")
            for j in joints_above:
                violations.append(f"J{j+1}={q_sol_deg[j]:.1f}° > max={self.q_max_deg[j]:.0f}°")
            cap_reason = f"Joint limit: {', '.join(violations)}"
            cap_details["violated_joints"] = list(joints_below) + list(joints_above)
            q_sol = self.q_ref.copy()  # Hold position
            q_sol_deg = q_sol * rad2deg
        
        # C. Cartesian Error Check (Predicted)
        T_final = self.ik_solver.robot.fkine(q_sol)
        sol_pos = np.array(T_final[:3, 3]).flatten()
        target_pos = np.array(target_pose[:3])
        target_rot = np.array(target_pose[3:6])  # rx, ry, rz in radians
        pred_err = np.linalg.norm(target_pos - sol_pos)
        pos_diff = target_pos - sol_pos
        
        # Get current pose for reference (from q_ref)
        T_current = self.ik_solver.robot.fkine(self.q_ref)
        current_pos = np.array(T_current[:3, 3]).flatten()
        R_current = np.array(T_current[0:3, 0:3])
        current_rot = self._rotation_matrix_to_euler(R_current)
        
        # Extract solved rotation from FK matrix
        R_mat = np.array(T_final[0:3, 0:3])
        sol_rot = self._rotation_matrix_to_euler(R_mat)
        rot_diff = target_rot - sol_rot
        rot_diff_deg = np.degrees(rot_diff)
        
        if pred_err > 0.3:  # 0.3m threshold - unreachable (tune as needed)
            status = "CAPPED"
            # Get current joint angles for reference
            q_current_deg = self.q_ref * rad2deg
            cap_reason = (
                f"Unreachable (err={pred_err:.4f}m > 0.3m)\n"
                f"         Current:   pos=[{current_pos[0]:+.4f}, {current_pos[1]:+.4f}, {current_pos[2]:+.4f}]m  rot=[{current_rot[0]:+.3f}, {current_rot[1]:+.3f}, {current_rot[2]:+.3f}]rad  grip={self.gripper_state:.2f}\n"
                f"         Target:    pos=[{target_pos[0]:+.4f}, {target_pos[1]:+.4f}, {target_pos[2]:+.4f}]m  rot=[{target_rot[0]:+.3f}, {target_rot[1]:+.3f}, {target_rot[2]:+.3f}]rad  grip={gripper:.2f}\n"
                f"         IK Joints: [{q_sol_deg[0]:+.1f}, {q_sol_deg[1]:+.1f}, {q_sol_deg[2]:+.1f}, {q_sol_deg[3]:+.1f}, {q_sol_deg[4]:+.1f}, {q_sol_deg[5]:+.1f}]°\n"
                f"         FK Verify: pos=[{sol_pos[0]:+.4f}, {sol_pos[1]:+.4f}, {sol_pos[2]:+.4f}]m  rot=[{sol_rot[0]:+.3f}, {sol_rot[1]:+.3f}, {sol_rot[2]:+.3f}]rad"
            )
            cap_details["current_pos_m"] = current_pos.tolist()
            cap_details["current_rot_rad"] = current_rot.tolist()
            cap_details["target_pos_m"] = target_pos.tolist()
            cap_details["target_rot_rad"] = target_rot.tolist()
            cap_details["ik_joints_deg"] = q_sol_deg.tolist()
            cap_details["fk_verify_pos_m"] = sol_pos.tolist()
            cap_details["fk_verify_rot_rad"] = sol_rot.tolist()
            cap_details["error_m"] = pred_err
            cap_details["pos_diff_m"] = pos_diff.tolist()
            q_sol = self.q_ref.copy()  # Hold position
            q_sol_deg = q_sol * rad2deg
        
        # D. Large Motion Check (Optional - just log, don't block for latency)
        motion_deg = np.max(np.abs(q_sol_deg - self.q_ref * rad2deg))
        max_motion_joint = np.argmax(np.abs(q_sol_deg - self.q_ref * rad2deg))
        if motion_deg > 10:  # >10 deg single step is unusual at 50Hz
            status = "WARN_LARGE_MOVE"
            q_current_deg = self.q_ref * rad2deg
            cap_reason = (
                f"Large motion: J{max_motion_joint+1} moves {motion_deg:.1f}° > 10°\n"
                f"         Current:   pos=[{current_pos[0]:+.4f}, {current_pos[1]:+.4f}, {current_pos[2]:+.4f}]m  rot=[{current_rot[0]:+.3f}, {current_rot[1]:+.3f}, {current_rot[2]:+.3f}]rad  grip={self.gripper_state:.2f}\n"
                f"         Target:    pos=[{target_pos[0]:+.4f}, {target_pos[1]:+.4f}, {target_pos[2]:+.4f}]m  rot=[{target_rot[0]:+.3f}, {target_rot[1]:+.3f}, {target_rot[2]:+.3f}]rad  grip={gripper:.2f}\n"
                f"         IK Joints: [{q_sol_deg[0]:+.1f}, {q_sol_deg[1]:+.1f}, {q_sol_deg[2]:+.1f}, {q_sol_deg[3]:+.1f}, {q_sol_deg[4]:+.1f}, {q_sol_deg[5]:+.1f}]°\n"
                f"         FK Verify: pos=[{sol_pos[0]:+.4f}, {sol_pos[1]:+.4f}, {sol_pos[2]:+.4f}]m  rot=[{sol_rot[0]:+.3f}, {sol_rot[1]:+.3f}, {sol_rot[2]:+.3f}]rad"
            )
        
        # 5. Execute Command (only if not in error state)
        if status in ["OK", "WARN_LARGE_MOVE"]:
            self.robot.rm_movej_canfd(list(q_sol_deg), 0, 0, 0, 0)  # Follow=0 (Low)
            self.q_ref = q_sol

        # 6. Get Telemetry (fast, non-blocking reads)
        ret_j, joints = self.robot.rm_get_joint_degree()
        ret_c, currents = self.robot.rm_get_current_joint_current()
        
        act_joints = joints if ret_j == 0 else [0]*6
        act_cur = currents if ret_c == 0 else [0]*6
        
        result = {
            "arm": self.arm_name,
            "status": status,
            "target_pose": target_pose,
            "sol_pose": list(sol_pos),
            "pred_err": pred_err,
            "motion_deg": motion_deg,
            "cmd_joints": list(q_sol_deg),
            "act_joints": act_joints,
            "act_currents": act_cur,
            "max_current": np.max(np.abs(act_cur)) if ret_c == 0 else 0
        }

        # 7. Gripper Logic (Executed AFTER motion command)
        cmd_grip = 1.0 if gripper > 0.5 else 0.0
        if cmd_grip != self.last_gripper_cmd:
            if cmd_grip > 0.5:
                self.robot.rm_set_gripper_pick_on(1000, 200, False, 0)
                self.gripper_state = 1.0
            else:
                self.robot.rm_set_gripper_release(1000, False, 0)
                self.gripper_state = 0.0
            self.last_gripper_cmd = cmd_grip
        
        # Add cap reason if present
        if cap_reason:
            result["cap_reason"] = cap_reason
            result["cap_details"] = cap_details
            
        return result

def init_vla(arms=None):
    """
    Initialize VLA controllers.
    args: arms (list of str): ['left_arm', 'right_arm']. Defaults to both.
    """
    global LEFT_CONTROLLER, RIGHT_CONTROLLER
    
    if arms is None:
        arms = ["left_arm", "right_arm"]
        
    success = True
    try:
        if "left_arm" in arms:
            LEFT_CONTROLLER = VLAArmController("left_arm")
        
        if "right_arm" in arms:
            RIGHT_CONTROLLER = VLAArmController("right_arm")
            
        return True
    except Exception as e:
        print(f"VLA Init Failed: {e}")
        return False

def left_arm_move(pose, gripper):
    if LEFT_CONTROLLER:
        return LEFT_CONTROLLER.step(pose, gripper)
    return {"status": "NOT_INIT"}

def right_arm_move(pose, gripper):
    if RIGHT_CONTROLLER:
        return RIGHT_CONTROLLER.step(pose, gripper)
    return {"status": "NOT_INIT"}

# --- Visualization Helper ---
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"

def print_dual_status(left_res, right_res):
    """Print colored dual-arm status with detailed telemetry."""
    def fmt(res, arm_label):
        if not isinstance(res, dict) or "status" not in res:
            return f"{RED}{arm_label}: INVALID{RESET}"
            
        s = res["status"]
        color = GREEN if s == "OK" else (YELLOW if s == "CAPPED" else RED)
        
        err = res.get("pred_err", 0) * 1000  # mm
        cur = res.get("max_current", 0)
        
        # Get first 3 joint angles for compact display
        joints = res.get("act_joints", [0]*6)
        j_str = f"J:[{joints[0]:.1f},{joints[1]:.1f},{joints[2]:.1f}...]"
        
        return f"{color}{arm_label} {s:<6}{RESET} Err:{err:>5.1f}mm Cur:{cur:>4.0f}mA {j_str}"

    l_str = fmt(left_res, "L")
    r_str = fmt(right_res, "R")
    
    print(f"{l_str}  |  {r_str}", flush=True)


# =============================================================================
# XVLA Client Integration Methods
# =============================================================================

def get_arm_state(arm_name: str = "right_arm") -> dict:
    """
    Get current arm state for proprio building.
    
    Args:
        arm_name: "left_arm" or "right_arm".
        
    Returns:
        Dict with pose, gripper, and joints.
    """
    controller = RIGHT_CONTROLLER if arm_name == "right_arm" else LEFT_CONTROLLER
    
    if controller is None:
        return {"status": "NOT_INIT", "pose": None, "gripper": 0.0, "joints": None}
        
    pose = controller.get_current_pose()
    return {
        "status": "OK",
        "pose": pose,
        "gripper": controller.gripper_state,
        "joints": list(controller.q_ref * rad2deg) if controller.q_ref is not None else None,
    }


def emergency_stop(arm_name: str = None) -> None:
    """
    Immediate halt for safe-stop.
    
    Args:
        arm_name: Specific arm or None for both.
    """
    if arm_name is None or arm_name == "left_arm":
        if LEFT_CONTROLLER:
            try:
                LEFT_CONTROLLER.robot.rm_set_arm_stop()
                print(f"{RED}[LEFT] Emergency stop commanded{RESET}")
            except Exception as e:
                print(f"{RED}[LEFT] Stop failed: {e}{RESET}")
                
    if arm_name is None or arm_name == "right_arm":
        if RIGHT_CONTROLLER:
            try:
                RIGHT_CONTROLLER.robot.rm_set_arm_stop()
                print(f"{RED}[RIGHT] Emergency stop commanded{RESET}")
            except Exception as e:
                print(f"{RED}[RIGHT] Stop failed: {e}{RESET}")


# Async Dual-Arm Coordination
import asyncio

async def step_async(arm_name: str, target_pose, gripper) -> dict:
    """
    Async wrapper for concurrent dual-arm execution.
    
    Args:
        arm_name: "left_arm" or "right_arm".
        target_pose: [x, y, z, rx, ry, rz] in meters/radians.
        gripper: 0.0-1.0.
        
    Returns:
        Status dict from step().
    """
    loop = asyncio.get_event_loop()
    controller = RIGHT_CONTROLLER if arm_name == "right_arm" else LEFT_CONTROLLER
    
    if controller is None:
        return {"status": "NOT_INIT", "arm": arm_name}
        
    return await loop.run_in_executor(None, controller.step, target_pose, gripper)


async def dual_arm_execute(
    left_pose, left_gripper,
    right_pose, right_gripper,
) -> tuple:
    """
    Execute both arms concurrently.
    
    Args:
        left_pose: Target pose for left arm.
        left_gripper: Gripper value for left arm.
        right_pose: Target pose for right arm.
        right_gripper: Gripper value for right arm.
        
    Returns:
        Tuple of (left_result, right_result).
    """
    left_task = asyncio.create_task(step_async("left_arm", left_pose, left_gripper))
    right_task = asyncio.create_task(step_async("right_arm", right_pose, right_gripper))
    
    return await asyncio.gather(left_task, right_task)
