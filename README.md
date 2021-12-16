# Assignment 2: Robot Operating System

The Robot Operating System (ROS) is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

### Installing and running

The ROS package contained in this repository has been developed and tested with [ROS Noetic 1.15.13](http://wiki.ros.org/noetic/Installation).</br>
Once ROS has beed installed, it is necessary to create a ROS workspace:

```bash
$ mkdir -p [workspace_name]/src
$ cd [workspace_name]/
$ catkin_make
```

A folder called "second_assignment" must be created inside the "src" folder.</br>
The files contained in this repository must be placed inside the folder just created.</br>
Now, it is necessary to rebuild the package by returning to the workspace folder and executing:

```bash
$ catkin_make
```

Remember that the .bashrc file can be opened with the following command:

```bash
$ gedit ~/.bashrc
```

The setup file of the workspace must be sourced by adding the following line at the end of the .bashrc file.

```
source [workspace_folder]/devel/setup.bash
```

Finally, it is possibile to run the simulation.</br>
The following commands must be executed in different terminals:

```bash
$ roscore
```
```bash
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
```bash
$ rosrun second_assignment robot_controller
```
```bash
$ rosrun second_assignment robot_ui
```

### Exercise
The objective of this assignment is create two nodes (robot\_controller and robot\_ui) to interract with ROS system to navigate a robot in the given circuit:</br>
- the robot\_controller node is used to actually control the robot movements by using some informations about the laser scanners embedded inside the robot to make it proceed forward while avoiding walls.
- the robot\_ui node is an interface that interacts with the user and makes him able to increase and descrease the robot speed and also reset the robot position.

### Algorithm pseudocode
- The robot\_controller node pseudocode is:
```
function "main()" :
1) initialize the node by calling the "ros::init" function.
2) create a Subscriber to the topic "/base_scan": the callback is "onEnvinronmentScan".
3) create a ServiceServer of the service "/speed_modifier": the callback is "onSpeedModified".
4) create a Publisher to the topic "/cmd_vel".
5) call "ros::spin".
```
```
function "onModifySpeed(req, res)" :
1) compute speed += req.speed_delta.
2) if speed < 0, set speed = 0.
3) set res.speed=speed.
```
```
function "onEnvinronmentScan(ranges)" :
1) compute valuesInSubsection=floor(ranges.size/5).
2) create scans array of size 5.
3) for i=0 to 4:
    4) compute start = i*valuesInSubsection.
    5) compute end = start+valuesInSubsection.
    6) find the minimum value of ranges between start and end.
    7) put the found minimum in scans[i].
8) call "moveRobotInCircuit(scans)".
```
```
function "moveRobotInCircuit(scans)" :
1) if speed is <= 0, return.
2) compute linear: increases if scans[2] increases.
3) map linear between 0 and speed.
4) for each remaining direction i:
    5) calculate weight_i: increases if scans[i] decreases.
    6) map weight_i between 0 and 2*angularSpeed.
7) compute angular = (sum weights i<0)-(sum weight i>0);
8) call "applyVelocityToRobot(linear, angular)".
```

```
function "applyVelocityToRobot(linear, angular)" :
1) create a Twist message.
2) set velocity.linear.x=linear.
3) set velocity.angular.z=angular.
4) send the message to "/cmd_vel".
```
- The robot\_ui node pseudocode is:
```
function "main()" :
1) initialize the node by calling the "ros::init" function.
2) create a ServiceClient for the service "/speed_modifier".
3) create a ServiceClient for the service "/reset_positions".
4) execute these instruction in loop:
    5) get a new command by calling "getIntFromConsole".
    6) if command==4 terminate execution.
    7) if command==1 call "resetRobotPosition".
    8) if command==2 call "modifyRobotSpeed(0.5F)".
    9) if command==3 call "modifyRobotSpeed(-0.5F)".
```
```
function "resetRobotPosition()" :
1) create an Empty message.
2) send the message to the "/reset_positions".
```
```
function "modifyRobotSpeed(delta)" :
1) create a SpeedModifier message called modifier.
2) set modifier.request.speed_delta=delta.
3) send the message to "/speed_modifier".
```
```
function "getIntFromConsole(bound0, bound1)" :
1) execute these instruction in loop:
    2) value=get a int from the console.
    3) if bound0 <= value <= bound1.
        4) return value.
```

### Algorithm explanation

