# Assignment 2: Robot Operating System

The Robot Operating System (ROS) is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

### Installing and running

The ROS package contained in this repository has been developed and tested with [ROS Noetic 1.15.13](http://wiki.ros.org/noetic/Installation).</br>
Once ROS has been installed, it is necessary to create a ROS workspace:

```bash
$ mkdir -p [workspace_name]/src
$ cd [workspace_name]/
$ catkin_make
```

Then, a folder called "second_assignment" need to be created inside the "src" folder.</br>
The files contained in this repository need to be placed inside the just created folder.</br>
Now, it is necessary to rebuild the package by moving to the workspace folder and executing:

```bash
$ catkin_make
```

NB. The .bashrc file can be opened with the following command:

```bash
$ gedit ~/.bashrc
```

The setup.bash file must be sourced so that ROS can find the workspace.<br>s
To do this, the following line must be added at the end of the .bashrc file:

```
source [workspace_folder]/devel/setup.bash
```

Finally, it is possible to run the simulation.</br>
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
The objective of this assignment is create two ROS nodes (here called _robot\_controller_ and _robot\_ui_) to interact with the ROS system to navigate a robot in the given circuit:</br>
- the _robot\_controller_ node is used to actually move the robot by using some information about laser scanners, embedded inside the robot, to make it proceed forward while avoiding walls.
- the _robot\_ui_ node is an interface that interacts with the user and makes him able to increase and decrease the robot speed and also reset the robot position.

### Algorithm pseudocode
- The _robot\_controller_ node pseudocode is:
```
function "main()" :
1) initialize the node by calling the "ros::init" function
2) create a Subscriber for "/base_scan": the callback is "onEnvinronmentScan"
3) create a ServiceServer for "/speed_modifier": the callback is "onSpeedModified"
4) create a Publisher for "/cmd_vel"
5) call "ros::spin"
```
```
function "onModifySpeed(req, res)" :
1) compute speed += req.speed_delta
2) if speed < 0, then set speed = 0
3) set res.speed = speed
```
```
function "onEnvinronmentScan(ranges)" :
1) compute valuesInSubsection = floor(ranges.size/5)
2) create an array called scans of size 5
3) for i=0 to 4:
    4) compute start = i*valuesInSubsection
    5) compute end = start+valuesInSubsection
    6) find the minimum value of ranges between start and end
    7) put the found minimum in scans[i]
8) call "moveRobotInCircuit(scans)"
```
```
function "moveRobotInCircuit(scans)" :
1) if speed is <= 0, return
2) compute linear: increases if scans[2] increases
3) map linear between 0 and speed
4) for each remaining direction i:
    5) calculate weight_i: increases if scans[i] decreases
    6) map weight_i between 0 and 2*angularSpeed
7) compute angular = (weights with i<0)-(weights with i>0)
8) call "applyVelocityToRobot(linear, angular)"
```

```
function "applyVelocityToRobot(linear, angular)" :
1) create a Twist message
2) set velocity.linear.x=linear
3) set velocity.angular.z=angular
4) send the message to "/cmd_vel"
```
- The _robot\_ui_ node pseudocode is:
```
function "main()" :
1) initialize the node by calling the "ros::init" function
2) create a ServiceClient for "/speed_modifier"
3) create a ServiceClient for "/reset_positions"
4) execute these instruction in loop:
    5) get a new command by calling "getIntFromConsole"
    6) if command==4 terminate execution
    7) if command==1 call "resetRobotPosition"
    8) if command==2 call "modifyRobotSpeed(0.5F)"
    9) if command==3 call "modifyRobotSpeed(-0.5F)"
```
```
function "resetRobotPosition()" :
1) create an Empty message
2) send the message to the "/reset_positions"
```
```
function "modifyRobotSpeed(delta)" :
1) create a SpeedModifier message called modifier
2) set modifier.request.speed_delta=delta
3) send the message to "/speed_modifier"
```
```
function "getIntFromConsole(bound0, bound1)" :
1) execute these instruction in loop:
    2) value=get a int from the console
    3) if bound0 <= value <= bound1 :
        4) return value
```

### Algorithm explanation
Between the two nodes, the one that requires a further explanation surely is the _robot\_controller_ node: <br>
The 'ranges' array obtained by the "/base\_scan" topic has a size of 720 values and each one of these values is a measure of the distance from the robot to the environment with a cone of vision of 180 degrees. To make the algorithm simpler, this array is divided into 5 subsections and the minimum value of each subsection is stored into another array called 'scans'.<br>

The 'scans' array can be accesses with indexes from 0 to 4 with the following meaning: <br>
- 0 : minimum value from -90° to -54° (RIGHT)<br>
- 1 : minimum value from -54° to -18° (FRIGHT)<br>
- 2 : minimum value from -18° to +18° (FRONT)<br>
- 3 : minimum value from +18° to +54° (FLEFT)<br>
- 4 : minimum value from +54° to +90° (LEFT)<br>

The rule used to make the robot move forward is the following one:<br>
```cpp
float linear = fmin(fmax(0, scans[FRONT]-1), 1) * speed;
```
It can be analized by considering small portions of it:
```cpp
float a = fmax(0, scans[FRONT]-1);
float b = fmin(fmax(0, scans[FRONT]-1), 1);
float c = fmin(fmax(0, scans[FRONT]-1), 1) * speed;
```
- **a** : if the value of scans\[FRONT\] (the distance in front of the robot) is less than 1, the result value is 0 and the robot does not move. Otherwise, the result value increases with the distance.
- **b** : the value of **b** is the value of **a** but limited to be more than 0 and less than 1.
- **c** : the value of **c** is the value of **b** but scaled with the value of 'speed'.

Similar considerations can be done in every other direction (RIGHT, FRIGHT, FLEFT, LEFT). But, instead of making the robot move forward, the resulting values make the robot turn in some direction.<br>
For example, if we consider scans\[RIGHT\], the value of 'wright' is the amount of turning that the robot should perform to avoid the obstacle on its right: the value increases if the distance from the obstancle on the right decreases.<br>
The rule used to make the robot rotate is to sum all the weights just calculated and considering that weights from directions on the left should be negative (to make the robot rotate to the right) and weights from directions on the right should be positive (to make the robot rotate on the left).<br>
```cpp
float angular = - wleft - wfleft + wfright + wright;
```

At this point, the values of 'linear' and 'angular' are obtained and the robot can be moved inside the circuit.
