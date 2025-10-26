#!/usr/bin/env python
# Python 2/3 compatibility imports
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from kinematics import *
# from onrobot_rg_control.msg import OnRobotRGOutput
# from onrobot_rg_control.srv import SetCommand, SetCommandResponse

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Checking state of the robots
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonJointState(object):
    def __init__(self):
        super(MoveGroupPythonJointState, self).__init__()
         ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_joint_state", anonymous=True)
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `PlanningSceneInterface`_ object.
        scene = moveit_commander.PlanningSceneInterface()
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )   
        # display_setonrobot = rospy.Publisher(
        #     'OnRobotRGOutput', OnRobotRGOutput, queue_size=2
        # )
        setonrobot = rospy.get_param('/onrobot/gripper', 'rg2')

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        print("group name=",group_name)
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL
         # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        #onrobot
        # self.display_setonrobot = display_setonrobot
        self.setonrobot = setonrobot

    def go_to_pose_goal(self, x,y,z, qx, qy, qz, qw):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_initial_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 4
        joint_goal[2] = tau / 4
        joint_goal[3] = -tau / 4
        joint_goal[4] = -tau / 4
        joint_goal[5] = -tau/6  # 1/6 of a turn
    

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # current_joints = move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)
    
    def go_my_ik(self,x,y,z,rx,ry,rz):
        move_group = self.move_group
        
        print("Connected to Movegroup")

        #Create Target Pose Matrix
        
        target_pos = [x, y, z] 
        
        target_rpy = [rx, ry, rz] 

        

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
    