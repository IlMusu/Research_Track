#! /usr/bin/env python

import rospy
import actionlib
import os

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# An ActionClient used to send commands to the move_base ActionServer
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
# A publisher to the /cmd_vel topic to move the robot
vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
# The velocity that needs to be checked when checking collision avoidance
desired_velocity = Twist()
    
def main():
    # Initializing node
    rospy.init_node("robot_ui")
    
    while 1:
        print("Choose one of the following commands:")
        print("1. Drive autonomously to a specific target.")
        print("2. Stop driving to specified target.")
        print("3. Drive manually without assistance.")
        print("4. Drive manually with collision avoidance.")
        print("5. Quit.")
        
        mode = input()
            
        if mode == "1" :
            set_specific_goal()
        elif mode == "2" :
            cancel_specific_goal()
        elif mode == "3" :
            drive_manually(False)
        elif mode == "4" :
            drive_manually(True)
        elif mode == "5" :
            return
        else :
            print("Invalid operation mode.")
            
        print()
    
def set_specific_goal():
    # Getting the target position from the user
    x = float(input("Insert x position: "))
    y = float(input("Insert y position: "))
    
    # Waiting until server exists
    client.wait_for_server()
    
    # Creating the action containing the target position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # Communicating the action to the server
    client.send_goal(goal)
    print("Setted a new target.") 
    
def cancel_specific_goal() :
    # Checking if a target has previously been set
    if client.get_state() == GoalStatus.LOST :
        print("No target is currently set.")
        return
    
    # Removing previously set target
    client.cancel_goal()
    print("Cleared target.")        
    
def drive_manually(assistance) :
    # Cancelling all goals to be sure move_base is stopped
    client.cancel_all_goals()
        
    if assistance :
        # Subscribing to the necessary topics for assisting robot movements
        sub1 = rospy.Subscriber("cmd_vel_override", Twist, on_manual_velocity)
        sub2 = rospy.Subscriber("scan", LaserScan, on_laser_scan)
        # Running teleop_twist_keyboard node to manually control robot
        # The default topic is overridden using a parameter: this could have also been done by 
        # creating a launch file and using the <remap> tag. Using the parameter makes it possible 
        # to have the desired behaviour without creating another file.
        os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel_override")
        # Unsubscribing from the topics
        sub1.unregister()
        sub2.unregister()
    else :
        # Running teleop_twist_keyboard node to manually control robot
        os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=cmd_vel")
        
    # Clearing robot movement
    vel_pub.publish(Twist())
   
    
def on_manual_velocity(velocity) :
    global desired_velocity
    desired_velocity = velocity
    
def on_laser_scan(scans) :
    # Dividing the scans into 3 directions: LEFT, FRONT, RIGHT
    # Checking if the minimum distance is more that 0.5
    clear_directions = [
        min(scans.ranges[  0:287]) > 0.5,
        min(scans.ranges[288:431]) > 0.5,
        min(scans.ranges[432:713]) > 0.5,
    ]
    
    # Check if the next movement would cause a collision and prevent it
    if desired_velocity.linear.x > 0 and not clear_directions[1] :
        desired_velocity.linear.x = 0
    if desired_velocity.angular.z > 0 and not clear_directions[2] :
        desired_velocity.angular.z = 0
    if desired_velocity.angular.z < 0 and not clear_directions[0] :
        desired_velocity.angular.z = 0
    
    # Publishing corrected velocity
    vel_pub.publish(desired_velocity)
    

if __name__ == '__main__':
    main()
    
