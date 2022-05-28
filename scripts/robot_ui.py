"""
.. module:: robot_ui
    :platform: Unix
    :synopsis: User Interface to control a robot.
.. moduleauthor:: Carmine Recchiuto carmine.recchiuto@dibris.unige.it

This ROS node is a User Interface which interacts with the ROS system to 
navigate a robot in the given environment. The node should let the user 
choose one of the following operation modes:\n
 1. Let the robot reach autonomously a target coordinate inserted by the user.\n
 2. Let the user drive manually the robot without assistance.\n
 3. Let the user drive manually the robot with collision avoidance.

Subscribes to:
    /cmd_vel_override
    /scan
    
Publishes to:
    /cmd_vel
    
Sends goals to:
    /move_base
    
"""
    
import rospy
import actionlib
import os

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

client = None
"""An ActionClient used to send commands to the move_base ActionServer"""

vel_pub = None
"""A publisher to the /cmd_vel topic to move the robot"""

desired_velocity = None
"""The velocity that needs to be checked when checking collision avoidance"""

def controller():
    """"
    This function initializes the ROS node and waits for the user to
    insert *start* or *stop* to control the robot, by relying on the
    `rospy <http://wiki.ros.org/rospy/>`_ module.
    """
    print("doing something")
    
    
def main():
    """
    |  This method initializes the ROS node with name 'robot_ui'.
    |  And then asks the user to choose one of the possible actions:
    |   1. Drive autonomously to a specific target.
    |   2. Stop driving to specified target.
    |   3. Drive manually without assistance.
    |   4. Drive manually with collision avoidance.
    |   5. Quit.
    |  If the requested action is valid, it starts to execute it.
    """
    global client
    global vel_pub
    global velocity
    
    rospy.init_node("robot_ui")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    velocity = Twist()

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
    """
    |  This method prompts the user to set a target position (x, y).
    |  It will then use the position to set a new goal on the 'move_base' server.
    """
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
    """
    |  This method cancels the current goal from the 'move_base' server.
    |  If the server does not currenly have any goal, the user is notified.
    """
    # Checking if a target has previously been set
    if client.get_state() == GoalStatus.LOST :
        print("No target is currently set.")
        return
    
    # Removing previously set target
    client.cancel_goal()
    print("Cleared target.")
    
def drive_manually(assistance) :
    """
    Args:
        assistance: a boolean which enables or disables the assistance logic.
        
    |  This method enables the user to drive the robot manually.
    |  It also enables collision avoidance assistance if requested:
    |  1. The Twist message is intercepted with a callback.
    |  2. Requests the scanner data with a subscriber.
    |  NB. The 'teleop_twist_keyboard' is used to actually drive the robot.
    """
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
    """
    Args:
        velocity : a Twist message that contains the new velocity.
        
    This is a method that lays between 'teleop_twist_keyboard' node and 'move_base': 
    when the collision avoidance assistance is requested, the Twist message sent
    by the 'teleop_twist_keyboard' is intercepted and stored to be used later when
    new data from the robot scanner is available.
    """
    global desired_velocity
    desired_velocity = velocity
    
def on_laser_scan(scans) :
    """
    Args:
        scans : a LaserScan message that contains the scanned data.
        
    |  A callback that is called when new data from the robot laser scanner is available.
    |  It divides the into 3 directions: LEFT, FRONT, RIGHT.
    |  For each direction, checks if the requested velocity would cause a collision and
    |  prevents it by zeroing the correct component in the velocity.
    |  Sends the corrected velocity to the 'move_base' action server.
    """
    
    # Dividing the scans into 3 directions: LEFT, FRONT, RIGHT
    # Checking if the minimum distance is more that 0.5
    clear_directions = [
        min(scans.ranges[  0:287]) > 0.5,
        min(scans.ranges[288:431]) > 0.5,
        min(scans.ranges[432:720]) > 0.5,
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
