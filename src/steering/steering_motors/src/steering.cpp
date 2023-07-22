/*
Main loop to interface the steering motors
    Loop Rate: 100 Hz




by Pablo
Last review: 2023/07/10

*/


#include <ros/ros.h>
#include <steering_motors.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    SteeringMotors motors(n);

    // Calibration
    motors.calibrationRoutine();

    while (ros::ok)
    {

        motors.setPos();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

