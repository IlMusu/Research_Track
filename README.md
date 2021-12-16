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
The pseudocode used to solve the exercise is the following.

Inside the robot\_controller node:
```
function "main()" :
1) initialize the node by calling the "ros::init" function.
2) create a Subscriber to the topic "/base_scan" to receive informations about the surrounding environment of the robot. When new data is available, the function "onEnvinronmentScan" is called.
3) create a Publisher to the topic "/cmd_vel" so that the robot can be moved by publishing a message containing a velocity.
4) create a ServiceServer of the service "/speed_modifier" to receive commands abount changing the velocity of the robot. When new commands are available, the function "onSpeedModified" is called.
5) call the function "ros::spin" to prevent the node from terminating the execution.
```
```
function "onModifySpeed(req, res)" :
1) the speed of the robot is updated by adding the value of req.speed_delta to it. If the resulting speed value is negative, it is set to 0.
2) the new speed of the robot is returned to the sender setting the value of res.speed.
```
```
function "onEnvinronmentScan(msg)" :
1) the msg->ranges field contains 720 values, these are subdivided into scans, an array of 5 elements, by taking the minimum value of each subsection.
2) the function "moveRobotInCircuit" is called by passing scans as argument.
```
```
function "moveRobotInCircuit(scans)" :
1) if the current speed is <= 0, the robot should not be moved, so return.
2) the linear speed is calculated by considering the distance in front of the robot: the speed should increase if the distance is increasing. The obtained value should be mapped between 0 and 1 and multiplied by the speed value.
3) calculate some weights in the remaining four directions: the weight should increase if the distance is decreasing. The obtained weight is mapped between 0 and 2*angularSpeed.
4) the angular speed is calculated as the sum of the weights: if the weight is on the left of the robot, it should be negative, otherwise it should be positive.
5) call the function "applyVelocityToRobot" by passing the linear and angular values just computed.
```

```
function "applyVelocityToRobot(linear, angular)" :
1) create a Twist message.
2) set velocity.linear.x=linear and velocity.angular.z=angular.
3) send the message.
```

Inside the robot\_controller node:
```
function "main()" :
1) initialize the node by calling the "ros::init" function.
2) create a ServiceClient for the service "/speed_modifier" to send informations about the desired change in the speed of the robot.
3) create a ServiceClient for the service "/reset_positions" to reset the position of the robot.
4) execute these instruction in loop:
    5) get a new command from the console by calling the function "getIntFromConsole".
    6) if command==4 terminate execution.
    7) if command==1 call the function "resetRobotPosition".
    8) if command==2 call the function "modifyRobotSpeed(0.5F)".
    9) if command==3 call the function "modifyRobotSpeed(-0.5F)".
```
```
function "resetRobotPosition()":
1) create an Empty message.
2) send it to the "/reset_positions" service.
```
```
function "modifyRobotSpeed(delta)":
1) create an SpeedModifier message called modifier.
2) set modifier.request.speed_delta=delta.
3) send the message.
```
```
function "getIntFromConsole(bound0, bound1)":
1) execute these instruction in loop:
    2) get a int from the console and call it value.
    3) if bound0 <= value <= bound1.
        4) return value.
```

### Algorithm explanation
The idea behind the algorithm is the following: the robot can see in front of itself with an angle of vision of 150 degrees. If the robot detects a wall too much close (walls are make up of gold tokens), it needs to decide whether to turn right or left and how much to turn. To decide the direction of turning, the robot checks the distance of the walls on it's right and left side and decides to turn in the direction on which the distance is lower. It turns until it's vision in free of obstacles. </br>
If, during the traveling, the robot detects a silver token near itself, the following routine takes place: the robot goes to the token and grabs it, rotates 180 degrees, releases the token and rotates 180 degrees again to return to the position it had before the beginning of the routine. The robot does not try to retake a just released silver token because it can only see silver tokens in front of it self.

### Future development
The procedure used to make the robot turn works most of the times, but it may happen that the robot turns in the wrong direction and starts backing up instead of going forward. This is currently solved by adjusting some parameters but a better solution would be to implement a more complex procedure for pathfinding.
