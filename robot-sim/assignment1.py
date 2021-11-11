from __future__ import print_function
from sr.robot import *
import time
import math

R = Robot()
"""instance of the class Robot"""

def drive(speed, seconds):
    """
    Function for setting a linear velocity.
    Args: 
    	speed (float): the speed of the wheels
	    seconds (float): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity.
    Args: 
    	speed (float): the speed of the wheels
	  	seconds (float): the time interval
    """
    R.motors[0].m0.power = -speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0
    
def find_token(marker_type, half_vision_angle, start_angle=0):
    """
    Function to find the closest token of the specified type.
    Only the token in the specified cone-of-vision are taken into account.
    Args:
    	marker_type: type of the marker of the token to find
		half_vision_angle (float - degrees): half angle of vision of the robot
		start_angle (float - degrees): angle that marks the direction to look
    Returns:
		dist (float): distance of the closest token (-1 if no token is detected)
		rot_y (float - degrees): angle between the robot and the token (-1 if no token is detected)
    """
    dist=100
    rot_y=0
    for token in R.see():
        if(token.info.marker_type is marker_type and
           token.dist < dist and 
           token.rot_y > start_angle-half_vision_angle and 
           token.rot_y < start_angle+half_vision_angle) :
            dist=token.dist
            rot_y=token.rot_y
            
    if dist==100:
        return -1, -1
        
    return dist, rot_y
   		
def avoid_walls():
    """
    Function that makes the robot avoid walls.
    Once a wall in front of the robot is found, it decides whether 
    to turn right of left based on what it has on its surroundings.
    """
    dist_r = find_token(MARKER_TOKEN_GOLD, 10, 90)[0]
    dist_l = find_token(MARKER_TOKEN_GOLD, 10, -90)[0]
    
    turnDirection = math.copysign(1, dist_l - dist_r)
    avoided_wall = False
    forward_steps = 0
    
    while True:
        # Checks for a golden token
        dist, rot_y = find_token(MARKER_TOKEN_GOLD, 75)
        
        if dist > 0 and dist < 0.8 :
            # Turns the robot according to the
            # orientation of the token
            turn(3*turnDirection, 0.5)
            avoided_wall = True
            forward_steps = 5
        elif forward_steps > 0 :
            # If robot just avoided wall, it goes forward 
            # to avoid entering some loops
            drive(30, 0.1)
            avoided_wall = True
            forward_steps -= 1
        else : 
            # Wall has been avoided, can return
            if avoided_wall :
                print("avoided wall")
            return
	
def go_and_perform_action(dist, rot_y, action):
    """
    Function that makes the robot move toward the target
    Args: 
    	dist (float): distance to the target
	    rot_y (float - degrees): robot's relative rotation to the target
    """
    # If robot has no target, returns
    if dist == -1 :
        return
		
    # If delta rotation is over positive threshold, we must rotate positive
    if rot_y > 2.0 :		
        turn(-2, 0.5)
    # If delta rotation is under negative threshold, we must rotate negative
    elif rot_y < -2.0 :	
        turn(+2, 0.5)
    # If distance to token is more than threshold, we must move forward	
    elif dist > 0.4 :
        drive(30, 0.1)
    # Else we arrived at target
    else :
        print("reached target token, performing action")
        action()
			
def put_token_behind():
	"""
	Function that makes the robot put the token behinf itself.
	There is some wait time between the rotations to make
	the accelleration of the weels near zero.
	"""
	print("grabbing and putting token behind")
	R.grab()
	drive(0, 2)
	turn(-20, 3)
	R.release()
	drive(-30, 1)
	drive(0, 2)
	turn(+20, 3)

def main():
    """
    Main function used to control the robot's behaviour.
    """
    while 1:
        # Robot moves in circuit while avoiding walls
        avoid_walls()
        
        # Gets next visible target
        target_dist, target_rot_y = find_token(MARKER_TOKEN_SILVER, 100)
     
        # Robots goes to the target if any is found
        if target_dist != -1 and target_dist <= 1:
            print("going to target")
            go_and_perform_action(target_dist, target_rot_y, put_token_behind)
        else :
            print("going forward")
            drive(30, 0.3)
        
# Here is the main function call
main()
	
