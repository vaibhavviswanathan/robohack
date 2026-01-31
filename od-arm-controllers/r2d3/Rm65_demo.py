from ik_qp import *
###########################  The following program is for RM65 parameter settings ####################
def RM65_Demo():

    dT = 0.033 # User data transmission cycle (transparent transmission cycle)
    # Be careful here, if your end-effector executes 1cm Cartesian space x, y, z displacement, note that, if your displacement is small about 1cm,
    # but your cycle is 0.005, then you get: 1cm/0.005 = 0.01m/0.005 = 2m/s. Since arm length is about 1m, making the end move 2m in 1s is obviously impossible.
    # You can refer to Teach Pendant -> Safety Config -> Max Linear Velocity is 0.25m/s. Please ensure your sent position divided by dT (velocity) does not exceed it.


    # Declare solver class, first parameter optional: ("RM65B","RM65SF","RM75B","RM75SF")
    robot = QPIK("RM65B", dT)

    # Set installation angle, work coordinate system and tool coordinate system, set according to actual situation
    robot.set_install_angle([0, 0, 0], 'deg') # Please fill in the teach pendant installation info here, note the unit!!!!!!!!
    robot.set_work_cs_params([0, 0, 0, 0, 0, 0, 0])
    robot.set_tool_cs_params([0, 0, 0, 0, 0, 0, 0])

    # Set joint limits, consistent with actual situation (optional), will use default if not set
    robot.set_joint_limit_max([ 178,  130,  135,  178,  128,  360], 'deg')  # Default is robot maximum/minimum joint limits
    robot.set_joint_limit_min([-178, -130, -135, -178, -128, -360], 'deg')

    # The following statement limits RM65 joint 3 angle, because straightening the arm belongs to boundary singularity.
    # Although our package can avoid it, I still don't recommend completely straightening the elbow, because
    # this will greatly restrict robot motion leading to large end pose error. Note carefully, if you set limits, make sure the robot's start config is within limits!!!!!!!!
    # Commented out by default, enable if needed
    # robot.set_6dof_elbow_min_angle(3, 'deg')

    # Joint velocity weight, this interface is provided in case you feel the robot moves too fast. Default is 1 (full speed).
    # Note that if set inappropriately small, it will cause tracking error,
    # because you are effectively limiting joint speed. If a certain speed is required to reach a target, limiting it may cause failure to reach.
    robot.set_dq_max_weight([1,1,1,1,1,1])

    # This is to adjust end pose error. For example, when avoiding singularity, if you want small x,y,z tracking error,
    # then decrease r,p,y weights. Consider other cases similarly.
    robot.set_error_weight([1, 1, 1, 1, 1, 1]) #(optional)

    # Demo code follows, please read
    sim_robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = sim_robot.rm_create_robot_arm("192.168.1.18", 8080)# Please match your IP here
    
    # Uncomment whichever demo to run, default is "Hui" character motion demo
    # "Hui" character motion part
    sim_robot.rm_movej([0, 25 , 90 , 0 , 65 , 0],20, 0, 0, 1)
    q_ref = np.array([0, 25, 90, 0 , 65 , 0]) * deg2rad
    Td = robot.fkine(q_ref)
    d = 40
    for i in range(4*d):
        start_time = time.time() 
        
        if( i < d):
            Td[0, 3] = Td[0, 3] + 0.002
        elif(i>=d and i<2*d):
            Td[1, 3] = Td[1, 3] + 0.002
        elif(i >= 2*d and i < 3*d):
            Td[0, 3] = Td[0, 3] - 0.002
        elif(i >= 3*d and i < 4*d):
            Td[1, 3] = Td[1, 3] - 0.002
      
        q_sol = robot.sovler(q_ref, Td)
            
        q_ref = q_sol
        q_sol = rad2deg*q_sol   # Note: q_sol is in radians, convert to degrees for transmission!!!!!!!!!!!!!!
    # "Hui" character motion part
#---------------------------------------------------------------------------------------------------#
    # Singularity avoidance part
    # sim_robot.rm_movej([0, 25 , 90 , -40.453 , 0 , 0], 20, 0, 0, True)
    # q_ref = np.array([0, 25, 90, -40.453 , 0 , 0]) * deg2rad
    # Td = robot.fkine(q_ref)
    # d = 80
    # for i in range(d):
    #     start_time = time.time() 
        
    #     if( i < d):
    #         Td[1, 3] = Td[1, 3] + 0.002

      
    #     q_sol = robot.sovler(q_ref, Td)
            
    #     q_ref = q_sol
    #     q_sol = rad2deg*q_sol   # Note: q_sol is in radians, convert to degrees for transmission!!!!!!!!!!!!!!
    # Singularity avoidance part

        sim_robot.rm_movej_canfd(q_sol,1,0,0,0)

        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time < dT:
            time.sleep(dT - elapsed_time)

if __name__ == '__main__':
    RM65_Demo()
