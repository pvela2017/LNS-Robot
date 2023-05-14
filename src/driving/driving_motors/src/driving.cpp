/*
Class to setup  RPMs of driving motors through TCP-CAN converter,
check rpm of each motor and speed based on the wheel diameter.
Also check controller alarm status and can clear the alarms.

Connect to socket 1

node: /driving_motors

Subscribe to: /driving_motors/commands
              /driving_motors/alarm_monitor/clear_alarm

Publish to: /driving_motors/alarm_monitor/status
            /driving_motors/feedback/rpm
            /driving_motors/feedback/speed

by Pablo
Last review: 2023/03/23
*/


#include <ros/ros.h>
#include <driving_motors.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    DrivingMotors motors(n);
    //motors.connSocket();


    while (ros::ok)
    {

        motors.setSpeed();
        motors.requestFeedback();
        motors.publishFeedback();

        //if (ros::isShuttingDown()) 
        //{
        //    motors.emergencyStop();
        //}

        ros::spinOnce();
        loop_rate.sleep();
    }

}

