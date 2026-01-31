# Teleoperation IK

**Author:**    Daryl/Ray/Leon

**Update:**   2024/09/29

### I. Install QP Solver Toolkit
After reading the following interface instructions, please move to `Rm65_demo` and carefully read every line of comments.

```shell
cd qp-tools\python 
pip install --no-build-isolation . 
```

If installation fails, please install the corresponding missing environment according to the prompts.

After installation, you can check if the tool is installed successfully via `import qpSWIFT`.


---

### II. Project Structure

- qp-tools:       Contains the QP solver toolkit
- ik_qp.py:       Used for defining the constrained inverse kinematics solver, including various parameter setting interfaces. Core file of this project.
- ik_rbtdef.py:   Defines MDH parameters for RM65/RM75 and other models, as well as common robotics algorithms. Modify DH parameters in this file if needed.
- ik_rbtutils.py: Defines utility functions for calculating pose difference, converting between quaternions/Euler angles and homogeneous transformation matrices, etc.
- ik_loadlib.py:  Defines functions related to loading dynamic link libraries, used for calling analytical inverse kinematics functions in DLLs. Currently disabled.
- demo.py:        Example program showing how to use `ik_qp.py` solver to calculate inverse kinematics during teleoperation.

---

### III. ik_qp.py Usage Instructions

1. Define a QPIK class before use:
   ```python
   robot = QPIK(type, dT)
   ```

   Input parameters: 
      - type: Robot type, optional parameters are ("RM65B","RM65SF","RM75B","RM75SF")
      - dT: User data transmission cycle (control cycle, must be consistent with the transparent transmission cycle), unit: second

2. Set Robot Parameters

   - Set Installation Angle:
      ```python
      robot.set_install_angle(angle)
      ```
      - Input parameters:
         - angle: Robot installation angle, unit: rad, default is [0, 0, 0]

   - Set Work Coordinate System:
      ```python
      robot.set_work_cs_params(pose)
      ```
      - Input parameters:
         - pose: Pose of work coordinate system relative to base coordinate system [x, y, z, r, p, y], unit: m\rad, default is [0, 0, 0, 0, 0, 0]

   - Set Tool Coordinate System:
      ```python
      robot.set_tool_cs_params(pose)
      ```
      - Input parameters:
         - pose: Pose of tool coordinate system relative to end-effector coordinate system [x, y, z, r, p, y], unit: m\rad, default is [0, 0, 0, 0, 0, 0]

3. Since the QP method allows for constrained solving, `ik_qp.py` provides interfaces to modify joint position limits and joint velocity limits during solving:

   - Set Maximum Joint Position Limit Interface:    
      ```python
         robot.set_joint_limit_max(angle) 
      ```

      - Input parameters:
         - angle: User-defined upper limit of joint range during solving, unit: rad, default is original robot joint upper limit

   - Set Minimum Joint Position Limit Interface:    
      ```python
      robot.set_joint_limit_min(angle)
      ```
   
      - Input parameters:
         - angle: User-defined lower limit of joint range during solving, unit: rad, default is original robot joint lower limit

   - Set RM65 Series Elbow Joint Maximum Position Limit Interface (Note: set in `set_joint_limit_max` and `set_joint_limit_min`):
      ```python
      robot.set_6dof_elbow_max_angle(angle)
      ```
      - Input parameters:
         - angle: User-defined upper limit of elbow joint range during solving, unit: rad, default is original robot elbow joint upper limit
      
   - Set RM65 Series Elbow Joint Minimum Position Limit Interface (Note: set in `set_joint_limit_max` and `set_joint_limit_min`):
      ```python
      robot.set_6dof_elbow_min_angle(angle)
      ```
      - Input parameters:
         - angle: User-defined lower limit of elbow joint range during solving, unit: rad, default is original robot elbow joint lower limit
   
   - Set RM75 Series Elbow Joint Maximum Position Limit Interface (Note: set in `set_joint_limit_max` and `set_joint_limit_min`):
      ```python
      robot.set_7dof_elbow_max_angle(angle)
      ```
      - Input parameters:
         - angle: User-defined upper limit of elbow joint range during solving, unit: rad, default is original robot elbow joint lower limit
      
   - Set RM75 Series Elbow Joint Minimum Position Limit Interface (Note: set in `set_joint_limit_max` and `set_joint_limit_min`):
      ```python
      robot.set_7dof_elbow_min_angle(angle)
      ```
      - Input parameters:
         - angle: User-defined lower limit of elbow joint range during solving, unit: rad, default is original robot elbow joint lower limit
         
   
   Through the above four interfaces, you can constrain the position range of the elbow joint for RM65 (corresponding to joint 3) and RM75 (corresponding to joint 4) during solving, making the movements more anthropomorphic during teleoperation.
   
   Taking the RM65 robot as an example, as shown in the figure below, the main operating area of the robot is the orange area in front of the chest. At this time, the elbow (blue circle) faces outward, and the joint 3 angle is greater than 0. If you want the elbow to always face outward, you should call `robot.set_6dof_elbow_min_angle(3 degrees)`. Whether to set the maximum to 3 degrees or the minimum to -3 degrees depends on the actual situation and the robot's installation method. The reason it is not set strictly to 0 degrees is to prevent the elbow from oscillating back and forth after the robot arm is straightened by user operation.
   
   For the RM75 robot, the elbow position is affected by the null space in addition to the joint 4 angle constraint. You can set the range of joint 3 according to the actual situation (joint 3 angle directly affects elbow position). Assuming the RM75 robot initial position is as shown below, and joint 3 angle is 0, you can set:
   
   ```python
   robot.set_7dof_q3_min_angle(-np.pi/6)
   robot.set_7dof_q3_max_angle(np.pi/6)
   ```
   
   This restricts joint 3 position to [-π/6, π/6], meaning the elbow always faces outward.


   <img src="img/Snipaste_2024-09-26_17-53-09.png" align="middle" width="80%"/>


   - Modify Joint Velocity Limit Interface:
      ```python
      robot.set_joint_velocity_limit(velocity)
      ```

      - Input parameters:
         - velocity: User-defined joint velocity limit during solving, unit: rad/s, default is original robot joint velocity limit

   - Set Joint Velocity Weight Interface:
      ```python
      robot.set_dq_max_weight(weight)
      ```

      - Input parameters:
         - weight: User-defined velocity weight, i.e., joint i's max velocity will be set to `(dq_max_new[i] = dq_max_now[i] * weight[i])`, range: 0~1. For 6DOF robots, joint 4 defaults to 0.1, others 1.0. For 7DOF, joint 5 defaults to 0.1, others 1.0. The purpose is to limit wrist velocity and prevent rapid wrist rotation caused by wrist singularities, but it needs to be tuned based on simulation tests.
      
      > **Note:** Since the joint velocity limit setting affects the solution results, it is recommended to constrain it according to actual needs. For example: when the given pose exceeds the robot's workspace, multiple solutions may cause the elbow joint to oscillate. In this case, you can constrain the elbow joint velocity weight to limit oscillation, but this may affect response speed in the normal workspace (since the elbow joint affects position most significantly).

4. Users can also set solver weights according to the actual situation:

   - Set Solver Weight Interface:
      ```python
      robot.set_error_weight(weight)
      ```
      - Input parameters:
         - weight: Solver weight, a 6-dimensional vector corresponding to end-effector pose x, y, z, r, p, y. The larger the weight, the more the solver focuses on converging that pose parameter error to 0. Range: 0~1, default is all-ones vector.

5. Solver Interface:

   ```python
   q_solve = robot.sovler(q_ref, Td)
   ```
   - Input parameters:
      - q_ref: Joint angles solved at the previous moment, unit: rad
      - Td: Expected 4x4 homogeneous transformation matrix of the robot end-effector at the current moment
   
   - Return value:
      - q_sovle: Solving result, unit: rad

---


### V. Other Notes

1. If the given pose exceeds the robot's workspace, the elbow joint (e.g., joint 3 for RM65) may oscillate slightly (multiple solutions problem), leading to unstable results. Therefore, it is recommended to limit the specific pose range sent to the robot as much as possible to avoid this.

2. Ensure that the QPIK DH parameters match the actual robot DH parameters, otherwise it may lead to incorrect results.

3. Ensure constraints are set correctly, otherwise it may lead to incorrect results. It is recommended to set simulation mode in the robot web teach pendant interface first. If teleoperation response is normal, then test on the real machine.

4. For periodic control, most common users directly use `sleep(dT)`, ignoring actual control program execution time. A suggested programming method is:

   ```python
   import time
   
   dT = 0.01 # 10ms
   while True:
      t_start = time.time()
      
      # Control Program
   
      t_end = time.time()
      t_sleep = dT - (t_end - t_start)
      if t_sleep > 0:
         time.sleep(t_sleep)
   ```
