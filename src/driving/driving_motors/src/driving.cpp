/*
Main loop to interface the driving motors
    Loop Rate: 30 Hz




by Pablo
Last review: 2023/07/10

*/


#include <ros/ros.h>
#include <driving_motors.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    DrivingMotors motors(n);


    while (ros::ok)
    {

        motors.setSpeed();
        motors.requestFeedback();
        motors.publishFeedback();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

