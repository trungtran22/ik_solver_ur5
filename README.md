# ik_solver_ur5
## Inverse Kinematics Solver and Motion Planning RRT for UR5
### Test on ROS 1 Noetic - Python 3
**Init UR5 model:**
- Init Moveit Planning and RViz:
  ```
  roslaunch ur5_moveit_config demo.launch
  ```
- Init UR5 inverse kinematics node and starting pick and place:
  ```
  rosrun ik_solver_ur5 main_ik.py
  ```
**Result**: 
- It's still not working nicely with the orientation, i'm trying to fix on that.
- Performing Pick and Place with the IK Solver and Moveit Planning.
- This result has not performed the RRT yet, only IK Solver.\
\
![](https://github.com/trungtran22/ik_solver_ur5/blob/main/pics/IMG_6565.GIF) 
