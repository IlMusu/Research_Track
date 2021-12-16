#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "second_assignment/SpeedModifier.h"

#define LaserScan sensor_msgs::LaserScan
#define SpeedModifier second_assignment::SpeedModifier
#define Twist geometry_msgs::Twist

#define RIGHT  0
#define FRIGHT 1
#define FRONT  2 
#define FLEFT  3
#define LEFT   4

void onEnvinronmentScan(const LaserScan::ConstPtr& msg);
bool onSpeedModified(SpeedModifier::Request &req, SpeedModifier::Response &res);
void moveRobotInCircuit(float ranges[]);
void applyVelocityToRobot(float linear, float angular);

ros::Subscriber robotLaserScanner;
ros::Publisher robotVelocityModifier;
ros::ServiceServer robotSpeedModifier;

float speed = 1.0F;

int main(int argc, char **argv)
{
    // Inits the node
    ros::init(argc, argv, "robot_controller");
    
    // This object is used for comunication with the ros system
    ros::NodeHandle nh;
    // From this topic we can receive scans about the environment  
    robotLaserScanner = nh.subscribe("/base_scan", 1, onEnvinronmentScan);
    // Creating a service to make it possibile to modify the speed of the robot
    robotSpeedModifier = nh.advertiseService("/speed_modifier", onSpeedModified);
    // Using this topic we can send the angular and linear velocity of the robot
    robotVelocityModifier = nh.advertise<Twist>("/cmd_vel", 1);
    
    // This makes the node not terminate
    ros::spin();
    
    return 0;
}

bool onSpeedModified(SpeedModifier::Request &req, SpeedModifier::Response &res)
{
    // Changes the speed with the given delta and bounds it to be non negative
    speed += req.speed_delta;
    if(speed < 0.0F)
        speed = 0.0F;
        
    // Returns to the client the current speed of the robot
    res.speed = speed;
    return true;
}

void onEnvinronmentScan(const LaserScan::ConstPtr& msg)
{
    int valuesInSubsection = floor(msg->ranges.size() / 5);
    
    // Subdividing environtment scans into some subsections
    // and then, finding the minimum for that subsection
    float scans[5];
    for(int i=0; i<5; ++i)
    {
        int start = i*valuesInSubsection;
        int end = start+valuesInSubsection;
        
        // Finding min for subsection
        float min = msg->ranges[start];
        for(int j=start+1; j<end; ++j)
            if(min > msg->ranges[j])
                min = msg->ranges[j];
            
        scans[i] = min;    
    }
    
    // Prints the current obtains scans
    ROS_INFO("Iteration scans: %f %f %f %f %f", scans[0], scans[1], scans[2], scans[3], scans[4]);
    
    moveRobotInCircuit(scans);
    
    printf("\n");
    fflush(stdout);
}

void moveRobotInCircuit(float scans[])
{
    if(speed <= 0.0F)
        return;
        
    float angularSpeed = 2.0F;
    
    // If the distance in front is less than 1, linear speed is 0
    // Otherwise, linear speed is bigger when distance in front is bigger
    float linear = fmin(fmax(0, scans[FRONT]-1), 1) * speed;
    
    // Calculates some weights for turning the robot
    // If the perceived distance is more than 2, that distance is ignored
    // Otherwise, the weight is bigger while the distance is smaller
    float wleft =   fmax(0.0F, 2-scans[LEFT])   * angularSpeed;
    float wfleft =  fmax(0.0F, 2-scans[FLEFT])  * angularSpeed;
    float wfright = fmax(0.0F, 2-scans[FRIGHT]) * angularSpeed;
    float wright =  fmax(0.0F, 2-scans[RIGHT])  * angularSpeed;
    float angular = - wleft - wfleft + wfright + wright;
    
    // It may happen that the weights sum up to be 0 because the robot
    // encounters a perfect simmetrical circuit, so this check makes it
    // turn randomly to find a new direction
    if(linear == 0 && angular == 0)
        angular = 0.3F;
        
    ROS_INFO("New velocity: linear %f | angular %f", linear, angular);
    
    applyVelocityToRobot(linear, angular);
}

void applyVelocityToRobot(float linear, float angular)
{
    // Create a message to control the velocity
    // of the robot with the arguments
    Twist velocity;
    velocity.linear.x = linear;
    velocity.angular.z = angular;
    
    // Publish the message to the topic
    robotVelocityModifier.publish(velocity);
}
