# Assignment 3: ROS, Gazebo And Rviz

The Robot Operating System (ROS) is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.
The full installation of ROS contains also Gazebo which is the most common 3-dimensional physical simulator used in robotics and Rviz which is a visualization widget for the simulation: together, these two tools allow the user to simulate a specific robot in a specific environment but also to view the simulated robot and environment, analyze log and replay sensor information.

### Installing and running

The ROS package contained in this repository has been developed and tested with [ROS Noetic 1.15.13](http://wiki.ros.org/noetic/Installation).</br>
Once ROS has been installed, it is necessary to create a ROS workspace:

```bash
$ mkdir -p [workspace_name]/src
$ cd [workspace_name]/
$ catkin_make
```

Then, a folder called "final_assignment" needs to be created inside the "src" folder.</br>
The files contained in this repository need to be placed inside the just created folder.</br>

Then, the package contained in the following repository needs to be added to the workspace.</br>
This package provides a laser-based SLAM (Simultaneous Localization and Mapping) algorithm. </br>
```
https://github.com/CarmineD8/slam_gmapping
```

Now, it is necessary to rebuild the package by moving to the workspace folder and executing:

```bash
$ catkin_make
```

NB. The .bashrc file can be opened with the following command:

```bash
$ gedit ~/.bashrc
```

The setup.bash file must be sourced so that ROS can find the workspace.<br>
To do this, the following line must be added at the end of the .bashrc file:

```
source [workspace_folder]/devel/setup.bash
```

Finally, it is possible to run the simulation.</br>
The following commands must be executed in different terminals:

```bash
$ roslaunch final_assignment simulation_gmapping.launch
```
```bash
$ roslaunch final_assignment move_base.launch
```
```bash
$ rosrun final_assignment robot_ui.py
```

### Exercise
The objective of this assignment is create a ROS nodes (here called _robot\_ui_) to interact with the ROS system to navigate a robot in the given environment with the following operation modes:</br>
1. Let the robot reach autonomously a target coordinate inserted by the user
2. Let the user drive manually the robot without assistance
3. Let the user drive manually the robot with collision avoidance

### Algorithm pseudocode
- The _robot\_ui_ node pseudocode is:
```
function "main()" :
1) initialize the node by calling "rospy.init_node()"
2) do the following operations in a loop:
    3) print the operation modes
    4) read the operation mode from console
    5) if mode == 1 call "set_specific_goal()"
    6) if mode == 2 call "cancel_specific_goal()"
    7) if mode == 3 call "drive_manually(False)"
    8) if mode == 4 call "drive_manually(True)"
    9) if mode == 5 exit loop
    10) else mode is invalid, print error
```
```
function "set_specific_goal()" :
1) read target position from console
2) create a goal with the target position
3) send the goal to the move_base action server

function "cancel_specific_goal()" :
1) if the move_base action server does not have a goal:
    2) return
3) cancel the goal from the move_base action server 
```
```
function "drive_manually(assistance)" :
1) cancel all goals from the move_base action server
2) if assistance :
    3) create subscriber for "/cmd_vel_override" : callback is "on_manual_velocity"
    4) create subscriber for "/scan" : callback is "on_laser_scan"
    5) run teleop_twist_keyboard and override the "/cmd_vel" topic
    6) unsubscribe from the "/cmd_vel_override" topic
    7) unsubscribe from the "/scan" topic
8) else :
    9) run teleop_twist_keyboard
10) reset robot velocity
```
```
function "on_manual_velocity(velocity)" :
1) desired_velocity = velocity
```
```
function "on_laser_scan(scans)" :
1) divide the scans.ranges array into 3 subsections
2) find the minimum of each subsection
3) put clear_directions[subsection] = (min for subsection > threshold)
4) if desired_velocity.linear.x > 0 and clear_directions[1] is false:
    5) desired_velocity.linear.x = 0
6) if desired_velocity.angular.z > 0 and clear_directions[2] is false:
    7) desired_velocity.angular.z = 0
8) if desired_velocity.angular.z < 0 and clear_directions[0] is false:
    9) desired_velocity.angular.z = 0
10) publish desired_velocity to the "/cmd_vel" topic
```

### Algorithm explanation

