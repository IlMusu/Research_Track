# Assignment 1: Python Robotics Simulator

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).</br>
Some of the arenas and the exercises have been modified for the Research Track I course.

### Installing and running

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/). Once the dependencies are installed, the simulation can be started with the following command inside the "robot-sim" folder: 

```bash
python2 run.py assignment1.py
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
    2) call "avoid_walls()" to avoid crashing the robot into a wall of gold tokens
    3) call "find_token()" to get the next visible silver token in front of the robot
    4) if there is a silver token near enough:
        5) call "go_and_perform_action(target, put_token_behind)"
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
1) find the distance from wall on the right by calling "find_token(MARKER_TOKEN_GOLD, 10, 90)"
2) find the distance from wall on the left by calling "find_token(MARKER_TOKEN_GOLD, 10, -90)"
3) turnDirection = left distance is more than the right distance ? 1 : -1
4) execute these instructions in a loop:
    5) check for a wall in front of the robot by calling "find_token(MARKER_TOKEN_GOLD, 75)"
    6) if there is a wall in front of the robot:
        7) turn with direction of turnDirection until there is no more a wall in front
        8) go forward a bit to avoid entering a loop of rotating left and right continuously
    9) otherwise return
```

### Algorithm explanation
The idea behind the algorithm is the following: the robot can see in front of itself with an angle of vision of 150 degrees. If the robot detects a wall too much close (walls are make up of gold tokens), it needs to decide whether to turn right or left and how much to turn. To decide the direction of turning, the robot checks the distance of the walls on it's right and left side and decides to turn in the direction on which the distance is lower. It turns until it's vision in free of obstacles. </br>
If, during the traveling, the robot detects a silver token near itself, the following routine takes place: the robot goes to the token and grabs it, rotates 180 degrees, releases the token and rotates 180 degrees again to return to the position it had before the beginning of the routine. The robot does not try to retake a just released silver token because it can only see silver tokens in front of it self.

### Future development
The procedure used to make the robot turn works most of the times, but it may happen that the robot turns in the wrong direction and starts backing up instead of going forward. This is currently solved by adjusting some parameters but a better solution would be to implement a more complex procedure for pathfinding.
