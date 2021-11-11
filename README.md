# Assignment 1: Python Robotics Simulator

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).</br>
Some of the arenas and the exercises have been modified for the Research Track I course.

### Installing and running

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).</br>
Once the dependencies are installed, the simulation can be started with the following command inside the "robot-sim" folder: 

```bash
$ python2 run.py assignment1.py
```

### Exercise
The objective of this assignment is to control the robot to make it navigate a circuit in a counterclockwise orientation. The walls are made of gold tokens and the robot must be able to see and avoid them while not backing up. Besides this, there are also some silver tokens inside the circuit which the robot should grab and put behind itself.

Some functions from previous exercises are slightly modified and used:
- drive(speed, seconds): sets a linear velocity to the robot for the specified time.
- turn(speed, seconds): sets an angular velocity to the robot for the specified time.

The pseudocode used to solve the exercize is the following:

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
6) otherwise, the robot arrived at distination:
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
        8) go forward a little bit to avoid entering a loop of rotating left and right continuosly
    9) otherwise return
```

### Future development
The procedure used to make the robot turn works most of the times, but it may happen that the robot turns in the wrong direction and starts backing up instead of goind forward. This is currently solved by adjusting some parameters but a better solution would be to implement a more complex procedure for pathfinding.
