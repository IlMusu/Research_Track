# Assignment 1: Python Robotics Simulator

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).</br>
Some of the arenas and the exercises have been modified for the Research Track I course.

### Installing and running

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).
Once the dependencies are installed, the simulation can be started with the following command: 

```bash
$ python2 run.py assignment1.py
```

### Exercise
The objective of this assignment is to control the robot to make it navigate a circuit in a counterclockwise orientation. The walls are made of gold tokens and the robot must be able to see and avoid them while not backing up. Besides this, there are also some silver tokens which the robot should grab and put behind.

In this assignment some function from the previous exercises are slightly modified and  used:
- drive(speed, seconds): this function sets a linear velocity to the robot for the specified time.
- turn(speed, seconds): this function sets an angular velocity to the robot for the specified time.

The pseudocode is the following:

```
function "main()":
1) execute this instructions in loop:
    2) call the function "avoid_walls()" to avoid the possibility of the robot crashing into gold tokens
    3) call the function "find_token()" to get the next visible silver token in front of the robot with a cone of vision of 200 degress
    4) if there is a silver token near enough:
        5) call the function "go_and_perform_action()" to make the robot go near the token and then perform the action "put_token_behind()"
    6) otherwise:
        7) call the function "drive()" to make the robot go forward a little bit
```
```
function "go_and_perform_action(dist, rot_y, action)" :
1) check if the arguments are valid, otherwise return
2) if the absolute value of rot_y is not inside a threshold:
    3) make the robot turn in order to make the robot face the target, hence, reduce the absolute value of rot_y
4) otherwise, if dist is more than a threshould: 
    5) make the robot go forward in order to reduce the distance from the target
6) otherwise, the robot arrived at distination so it possibile to perform the action passed a argument
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
1) find the distance to the nearest golden token on the right side by calling the function "find_token" with a small angle of vision and a start angle of 90 degrees
2) find the distance to the nearest golden token on the left side by calling the function "find_token" with a small angle of vision and a start angle of -90 degrees
3) compute the optimal direction turnDirection=1 if the left distance is more than the right distance, turnDirection=-1 otherwise
4) execute these instructions in a loop:
    5) check for a golden token in front of the robot by calling the function "find_token" with a vision angle of 150 degrees.
    6) if there is a golden token in front of the robot:
        7) turn by the direction specified by turnDirection until there is no more a golden token in front
        8) go forward a little bit to avoid entering a loop of rotating left and right continuosly
    9) otherwise:
        10) return from function
