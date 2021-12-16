#include "ros/ros.h"
#include "second_assignment/SpeedModifier.h"
#include "std_srvs/Empty.h"

#define SpeedModifier second_assignment::SpeedModifier
#define Empty std_srvs::Empty

int getIntFromConsole(int bound0, int bound1);
void resetRobotPosition();
void modifyRobotSpeed(float delta);

ros::ServiceClient robotSpeedControls;
ros::ServiceClient robotResetPosition;

int main(int argc, char **argv)
{
    // Inits the node
    ros::init(argc, argv, "robot_ui");
    
    // This object is used for comunication with the ros system
    ros::NodeHandle nh;
    // Using this topic we can change the robot speed and reset its position
    robotSpeedControls = nh.serviceClient<SpeedModifier>("/speed_modifier", 1);
    // Using this service we can reset the robot position
    robotResetPosition = nh.serviceClient<Empty>("/reset_positions");
    
    printf("Insert these commands to control the robot:\n");
    printf("1 : Reset robot position and orintation\n");
    printf("2 : Increase the robot's speed\n");
    printf("3 : Decrease the robot's speed\n");
    printf("4 : Quit\n\n");
    fflush(stdout);
    
    while(1)
    {
        // The int returned can only be between [1,4]
        int command = getIntFromConsole(1, 4);
        
        if(command == 4)
            break;
            
        switch(command)
        {
        case 1:
            resetRobotPosition();
            break;
        case 2:
            modifyRobotSpeed(0.5F);
            break;
        case 3: 
            modifyRobotSpeed(-0.5F);
        }
    }
}

void resetRobotPosition()
{
    Empty empty;
    
    if(!robotResetPosition.call(empty))
        ROS_ERROR("Failed to call service /reset_positions");
    else
        ROS_INFO("Robot position reset.");
}

void modifyRobotSpeed(float delta)
{
    SpeedModifier modifier;
    modifier.request.speed_delta = delta;   
         
    if(!robotSpeedControls.call(modifier))
        ROS_ERROR("Failed to call service /speed_modifier");
    else
        ROS_INFO("Robot current speed is %f.", modifier.response.speed);
}

int getIntFromConsole(int bound0, int bound1)
{
    int value;
    
    char line[80];
    while(1)
    {
        scanf("%s", line);
        value = atoi(line);
        
        if(bound0 <= value && value <= bound1)
            break;
            
        printf("Invalid input!\n");
        fflush(stdout);
    }
    
    return value;
}

