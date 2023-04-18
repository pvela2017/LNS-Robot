/*
Class to setup  RPMs of steering motors through TCP-CAN converter,
check rpm of each motor.
Also check controller alarm status and can clear the alarms.

Connect to socket 2

node: /steering_motors

Subscribe to: /steering_motors/commands
              /steering_motors/alarm_monitor/clear_alarm

Publish to: /steering_motors/alarm_monitor/status
            /steering_motors/feedback/rpm

by Pablo
Last review: 2023/03/30

*/


#include <ros/ros.h>
#include <steering_motors.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_motors");
    ros::NodeHandle n, n1, n2, n3, n4;
    ros::Rate loop_rate(30);

    SteeringMotors motors(n, n1, n2, n3, n4);
    motors.connSocket();

    // Calibration
    //motors.calibrationRoutine();

    while (ros::ok)
    {

        //motors.alarmMonitor();
        motors.feedback();
        motors.spinners();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

