/*
Class to setup  RPMs of steering motors through TCP-CAN converter,
check rpm of each motor and speed based on the wheel diameter.
Also check controller alarm status and can clear the alarms.

Connect to socket 2

node: /steering_motors

Subscribe to: /steering_motors/commands
              /steering_motors/alarm_monitor/clear_alarm

Publish to: /steering_motors/alarm_monitor/status
            /steering_motors/feedback/rpm
            /steering_motors/feedback/speed

by Pablo
Last review: 2023/03/28
*/


#include <ros/ros.h>
#include <steering_motors.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    SteeringMotors motors(n);
    motors.connSocket();


    while (ros::ok)
    {

        //motors.alarmMonitor();
        motors.feedback();

        if (ros::isShuttingDown()) 
        {
            motors.emergencyStop();
        }

        motors.spinner();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

