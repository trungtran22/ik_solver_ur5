#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
import sys
from kinematics import *


def main():
    # Init ROS node
    rospy.init_node('ur5_ik_controller_node', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    print("Connected to Movegroup")

    # Create Target Pose Matrix
    
    target_pos = [-0.5, -0.5, 0.6] 
    
    target_rpy = [np.pi/2, np.pi/2, 0] 

    print(f"Creating T_target: {target_pos}")
    T_target = create_target_pose(target_pos, target_rpy)


    print("Calculating IK...")
    d_params = [0.089159, 0.10915, 0.09465, 0.0823]
    a_params = [-0.425, -0.39225]
    
    try:
        solutions = solve_ik(T_target, d_params, a_params)
        if not solutions:
            print("Your IK solver returned no solutions.")
            return
    except Exception as e:
        print(f"Errors in your code: {e}")
        return

    q_goal = solutions[0] 
    print(f"Your IK has found solution: {np.rad2deg(q_goal)}")

    # Execute the planned trajectory with MoveIt
    
    print("Requesting planning and executing movement from MoveIt...")
    
    move_group.set_joint_value_target(q_goal)
    
    # Help with 3D collision and RRT from MoveIt
    move_group.go(wait=True) 
    
    print("Finished movement!")
    
    move_group.stop()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

