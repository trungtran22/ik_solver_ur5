#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import sys
from kinematics import *
from move_group_joint_state import MoveGroupPythonJointState

pi = np.pi
# group_name = "manipulator"
# move_group = moveit_commander.MoveGroupCommander(group_name)
# print("Connected to Movegroup")

# def go(x,y,z,roll,pitch,yaw):

    
    # # Create Target Pose Matrix
    # target_pos = [x, y, z] 
    
    # target_rpy = [roll, pitch, yaw] 

    # print(f"Creating T_target: {target_pos}")
    # T_target = create_target_pose(target_pos, target_rpy)


    # print("Calculating IK...")
    # d_params = [0.089159, 0.10915, 0.09465, 0.0823]
    # a_params = [-0.425, -0.39225]
    
    # try:
    #     solutions = solve_ik(T_target, d_params, a_params)
    #     if not solutions:
    #         print("Your IK solver returned no solutions.")
    #         return
    # except Exception as e:
    #     print(f"Errors in your code: {e}")
    #     return

    # q_goal = solutions[0] 
    # print(f"Your IK has found solution: {np.rad2deg(q_goal)}")

    # # Execute the planned trajectory with MoveIt
    
    # print("Requesting planning and executing movement from MoveIt...")
    
    # move_group.set_joint_value_target(q_goal)
    # move_group.go(wait=True) 
    
    # print("Finished movement!")
    
    # move_group.stop()
    # moveit_commander.roscpp_shutdown()

def main():
    run = MoveGroupPythonJointState()

    run.go_to_initial_state()
    # Pick
    run.go_my_ik(-0.5,0.19,0.5,pi,0,0)
    run.go_my_ik(-0.5,0.19,.33,pi,0,0)
    run.go_my_ik(-0.5,0.19,0.5,pi,0,0)
    # Place
    run.go_my_ik(-0.5,0.3,0.5,pi,0,0)
    run.go_my_ik(-0.5,0.3,0.33,pi,0,0)
    run.go_my_ik(-0.5,0.3,0.5,pi,0,0)

    run.go_to_initial_state()
    # Init ROS node
    # rospy.init_node('ur5_ik_controller_node', anonymous=True)
    # moveit_commander.roscpp_initialize(sys.argv)

    # #Pick

    # print("Picking...")
    # go(0,0,0.25,pi,0,0)
    # go(0,0,0.1,pi,0,0)
    # go(0,0,0.25,pi,0,0)

    # #Place
    
    # print("Placing...")
    # go(0.4,-0.2,0.25,pi,0,0)
    # go(0.4,-0.2,0.1,pi,0,0)
    # go(0.4,-0.2,0.25,pi,0,0)
    # group_name = "manipulator"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # print("Connected to Movegroup")

    # #Create Target Pose Matrix
    
    # target_pos = [-0.9, 0, 0.6] 
    
    # target_rpy = [pi, 0,0] 

    

    # print(f"Creating T_target: {target_pos}")
    # T_target = create_target_pose(target_pos, target_rpy)


    # print("Calculating IK...")
    # d_params = [0.089159, 0.10915, 0.09465, 0.0823]
    # a_params = [-0.425, -0.39225]
    
    # try:
    #     solutions = solve_ik(T_target, d_params, a_params)
    #     if not solutions:
    #         print("Your IK solver returned no solutions.")
    #         return
    # except Exception as e:
    #     print(f"Errors in your code: {e}")
    #     return

    # q_goal = solutions[0] 
    # print(f"Your IK has found solution: {np.rad2deg(q_goal)}")

    # # Execute the planned trajectory with MoveIt
    
    # print("Requesting planning and executing movement from MoveIt...")
    
    # move_group.set_joint_value_target(q_goal)
    
    # # Help with 3D collision and RRT from MoveIt
    # move_group.go(wait=True) 
    
    # print("Finished movement!")
    
    # move_group.stop()
    # moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

