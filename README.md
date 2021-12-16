# Assignment 2: Robot Operating System

The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.

### Installing and running

The following code has been developed and tested with [ROS Noetic 1.15.13](http://wiki.ros.org/noetic/Installation).</br>
Once this package has beed installed, it is necessary to create a ROS workspace:

```bash
$ mkdir -p [workspace_name]/src
$ cd [workspace_name]/
$ catkin_make
```

A folder called "second_assignment" must be created inside the "src" folder.</br>
The files contained in this repository must be placed inside the folder just created.</br>
Now it is necessary to rebuild the package by returning to the workspace folder and executing:

```bash
$ catkin_make
```

It is necessary to source the setup file by adding the following line to the .bashrc file.
NB. The file must be added at the end of the file.

```
source [workspace_folder]/devel/setup.bash
```

The .bashrc file can be opened with the following command:

```bash
$ gedit ~/.bashrc
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
The objective of this assignment is to control the robot to make it navigate a circuit in a counterclockwise orientation. The walls are made of gold tokens and the robot must be able to see and avoid them while not backing up. Besides this, there are also some silver tokens inside the circuit which the robot should grab and put behind itself.

Some functions from previous exercises are slightly modified and used:
- drive(speed, seconds): sets a linear velocity to the robot for the specified time.
- turn(speed, seconds): sets an angular velocity to the robot for the specified time.

### Algorithm pseudocode
The pseudocode used to solve the exercise is the following:

```
function "main()" :
1) execute this instructions in a loop:
    2) call "avoid_walls()" to avoid crashing the robot into the walls of gold tokens
    3) call "find_token()" to get the next visible silver token in front of the robot
    4) if there is a silver token near enough:
        5) call "go_and_perform_action(target, put_token_behind)" to make the robot go near the silver token and then perform the action
    6) otherwise:
        7) call "drive()" to make the robot go forward a little bit
```
```
function "go_and_perform_action(dist, rot_y, action)" :
1) check if the arguments are valid, otherwise return
2) if the absolute value of rot_y is not inside a threshold:
    3) make the robot turn in order to make it face the target
4) otherwise, if dist is more than a threshold: 
    5) make the robot go forward in order to reduce the distance from the target
6) otherwise, the robot arrived at destination:
    7)  perform the action passed a argument
```
```
function "grab_and_put_token_behind()":
1) grab the token
2) turn 180 degrees
3) release the token
4) move back a little bit to avoid colliding with the token when turning
5) turn 180 degrees
```
```
function "avoid_walls()":
1) find the distance to the nearest golden token on the right side by calling "find_token()" with a small angle of vision and a start angle of 90 degrees
2) find the distance to the nearest golden token on the left side by calling "find_token()" with a small angle of vision and a start angle of -90 degrees
3) compute the optimal direction turnDirection=1 if the left distance is more than the right distance, otherwise turnDirection=-1
4) execute these instructions in a loop:
    5) check for a golden token in front of the robot by calling "find_token()" with a vision angle of 150 degrees.
    6) if there is a golden token in front of the robot:
        7) turn by the direction specified by turnDirection until there is no more a golden token in front
        8) go forward a little bit to avoid entering a loop of rotating left and right continuously
    9) otherwise return
```

### Algorithm explanation
The idea behind the algorithm is the following: the robot can see in front of itself with an angle of vision of 150 degrees. If the robot detects a wall too much close (walls are make up of gold tokens), it needs to decide whether to turn right or left and how much to turn. To decide the direction of turning, the robot checks the distance of the walls on it's right and left side and decides to turn in the direction on which the distance is lower. It turns until it's vision in free of obstacles. </br>
If, during the traveling, the robot detects a silver token near itself, the following routine takes place: the robot goes to the token and grabs it, rotates 180 degrees, releases the token and rotates 180 degrees again to return to the position it had before the beginning of the routine. The robot does not try to retake a just released silver token because it can only see silver tokens in front of it self.

### Future development
The procedure used to make the robot turn works most of the times, but it may happen that the robot turns in the wrong direction and starts backing up instead of going forward. This is currently solved by adjusting some parameters but a better solution would be to implement a more complex procedure for pathfinding.
