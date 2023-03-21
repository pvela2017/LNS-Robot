
#include <ros/ros.h>
#include <driving_motors.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    DrivingMotors motors(n);
    motors.connSocket();


    while (ros::ok)
    {
        // Check alarms for all the motors
        for (int i = 1; i < 5; i++)
        {
            motors.alarmMonitor(motors.motorID[i]);
        }
           
        ros::spinOnce();
        loop_rate.sleep();
    }

}