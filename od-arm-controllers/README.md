# Open Droids Controllers

**Created by [SampreethRadhaKrishna](https://github.com/SampreethRadhaKrishna) @ [Open Droids](https://github.com/Open-Droids-robot)**

A Python-based robot arm control system for open droids robots , featuring a high-performance QP-based inverse kinematics solver and dual-arm coordination support. USE THIS ONLY FOR CARTESIAN CORRDINATE POSITION CONTROL. DO NOT USE THIS FOR JOINT LEVEL CONTROL.

There is a simpler solution for joint level control, which is to use the RealMan Python SDK directly. use movejcanfd() function from the SDK to move the arm in joint level control.

## Features

- **Quadratic Programming (QP) IK Solver** - Constrained inverse kinematics with joint limits and velocity constraints
- **Forward Kinematics** - Compute end-effector poses from joint angles
- **Dual-Arm Support** - Concurrent control of left and right arms via async execution
- **Safety Checks** - NaN detection, joint limit validation, reachability verification
- **Gripper Control** - Integrated pick/release operations

## Directory Structure

```
open_droids_controllers/
├── r2d3/
│   ├── VLA_move.py          # Main arm controller (high-level API)
│   ├── fk_test.py           # Forward kinematics testing utility
│   ├── ik_test.py           # Inverse kinematics testing utility
│   ├── robot_config.yaml    # Robot configuration (IPs, mounting, tool offsets)
│   ├── ik_qp.py             # QP-based IK solver (core)
│   ├── ik_rbtdef.py         # Robot MDH parameters & algorithms
│   ├── ik_rbtutils.py       # Utility functions (pose conversions)
│   ├── ik_loadlib.py        # DLL loading utilities (optional)
│   ├── kinematics_usage.md  # IK solver documentation
│   ├── qp-tools/            # QP solver toolkit
│   └── lib/                 # Shared libraries
└── RM_API2-1.0.6/           # RealMan Python SDK
```

## Installation

### 1. Install QP Solver Toolkit

```bash
cd r2d3/qp-tools/python
pip install --no-build-isolation .
```

Verify installation:
```python
import qpSWIFT  # Should import without errors
```

### 2. Install Dependencies

```bash
pip install numpy pyyaml
```

---

## Robot Configuration (`robot_config.yaml`)

Defines per-arm settings including network, mounting, and tool coordinate systems.

```yaml
left_arm:
  ip: "169.254.128.18"       # Robot IP address
  port: 8080                  # Communication port
  install_angle: [0, 45, 0]   # Mounting rotation [rx, ry, rz] in degrees
  work_cs: [0, 0, 0, 0, 0, -3.14159]  # Work coordinate system [x,y,z,rx,ry,rz] (meters/radians)
  tool_cs: [0, 0, 0, 0, 0, 0]         # Tool center point offset

right_arm:
  ip: "169.254.128.19"
  port: 8080
  install_angle: [0, -45, 0]
  work_cs: [0, 0, 0, 0, 0, 3.14159]
  tool_cs: [0, 0, 0.135, 0, 0, 0]     # 135mm tool offset in Z
```

### Configuration Parameters

| Parameter | Description | Units |
|-----------|-------------|-------|
| `ip` | Robot controller IP address | - |
| `port` | TCP port for communication | - |
| `install_angle` | Robot base mounting orientation | degrees |
| `work_cs` | Work coordinate system relative to base | m, rad |
| `tool_cs` | Tool center point relative to end flange | m, rad |

---

## Usage

### VLA_move.py - Main Controller

The primary interface for robot arm control with full IK solving and safety checks.

#### Initialize Arms

```python
from VLA_move import init_vla, right_arm_move, left_arm_move, get_arm_state

# Initialize both arms
init_vla()

# Or initialize specific arm
init_vla(["right_arm"])
```

#### Execute Motion

```python
# Target pose: [x, y, z, rx, ry, rz] in meters and radians
target_pose = [0.3, 0.1, 0.25, 3.14, 0, 0]
gripper = 0.0  # 0.0 = open, 1.0 = closed

result = right_arm_move(target_pose, gripper)
print(result["status"])  # "OK", "CAPPED", "ERR_JOINT_LIMIT", etc.
```

#### Get Current State

```python
state = get_arm_state("right_arm")
print(f"Pose: {state['pose']}")      # [x, y, z, rx, ry, rz]
print(f"Gripper: {state['gripper']}") # 0.0 or 1.0
print(f"Joints: {state['joints']}")   # [j1, j2, j3, j4, j5, j6] degrees
```

#### Dual-Arm Async Execution

```python
import asyncio
from VLA_move import init_vla, dual_arm_execute

init_vla()

left_pose = [0.3, 0.2, 0.25, 0, 0, 0]
right_pose = [0.3, -0.2, 0.25, 0, 0, 0]

# Execute both arms concurrently
left_result, right_result = asyncio.run(
    dual_arm_execute(left_pose, 0.0, right_pose, 1.0)
)
```

#### Emergency Stop

```python
from VLA_move import emergency_stop

emergency_stop()              # Stop both arms
emergency_stop("right_arm")   # Stop specific arm
```

---

### fk_test.py - Forward Kinematics

Compute Cartesian pose from joint angles.

```python
from fk_test import FKTester
import numpy as np

fk = FKTester(robot_type="RM65B")

# Joint angles in radians
joints_rad = [0.0, 0.5, -0.3, 0.0, -1.5, 0.0]
result = fk.compute_fk(joints_rad)

print(f"Position: {result['position']}")       # [x, y, z] meters
print(f"Euler: {result['euler_rad']}")         # [rx, ry, rz] radians
print(f"Quaternion: {result['quaternion']}")   # [w, x, y, z]
```

#### Run Standalone Test

```bash
cd r2d3
python fk_test.py
```

---

### ik_test.py - Inverse Kinematics Testing

Interactive tool for testing IK solutions with live robot connection.

```python
from ik_test import IKTester

# Connect to robot
tester = IKTester(
    ip="169.254.128.18",
    port=8080,
    robot_type="RM65B",
    install_angle=[0, 45, 0],
    work_cs=[0, 0, 0, 0, 0, -1.571],
    tool_cs=[0, 0, 0, 0, 0, 0]
)

# Get current pose
current = tester.get_current_pose()
print(f"Current: {current}")

# Test IK and move to target
target = [0.3, 0.1, 0.25, 3.14, 0, 0]  # [x, y, z, rx, ry, rz]
tester.test_ik_move(target)

# Cleanup
tester.disconnect()
```

#### Run Standalone Test

Edit the target pose in the script, then:

```bash
cd r2d3
python ik_test.py
```

---

## IK Solver Configuration

The QP-based solver supports extensive configuration. See `kinematics_usage.md` for full documentation.

### Key Concepts

```python
from ik_qp import QPIK

# Initialize solver
solver = QPIK("RM65B", dT=0.02)  # 50Hz control rate

# Set mounting orientation
solver.set_install_angle([0, 45, 0], 'deg')

# Set work coordinate frame
solver.set_work_cs_params([0, 0, 0, 0, 0, -1.571])

# Set tool offset
solver.set_tool_cs_params([0, 0, 0.1, 0, 0, 0])

# Configure joint limits
solver.set_joint_limit_min([-178, -130, -135, -178, -128, -360], 'deg')
solver.set_joint_limit_max([178, 130, 135, 178, 128, 360], 'deg')

# Solve IK
from ik_rbtutils import pose_to_matrix
target_pose = [0.3, 0.1, 0.25, 0, 0, 0]
Td = pose_to_matrix(target_pose)

q_current = [0, 0, 0, 0, 0, 0]  # Initial seed (radians)
q_solution = solver.sovler(q_current, Td)
```

---

## Motion Status Codes

| Status | Description |
|--------|-------------|
| `OK` | Motion executed successfully |
| `CAPPED` | Target unreachable, position held |
| `ERR_JOINT_LIMIT` | IK solution violates joint limits |
| `ERR_NAN` | IK solver produced invalid result |
| `WARN_LARGE_MOVE` | Motion >10° per step (logged only) |
| `NOT_INIT` | Arm controller not initialized |

---

## Supported Robot Models

- RM65B / RM65SF (6-DOF)
- RM75B / RM75SF (7-DOF)

---

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- RealMan Robotics for the RM API
- QP-SWIFT solver team
