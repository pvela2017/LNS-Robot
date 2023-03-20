
#include <ros/ros.h>
#include <driving_motors.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::Rate loop_rate(1);

    DrivingMotors motors;
    motors.connSocket();


    while (ros::ok)
    {
        motors.setSpeed();
        loop_rate.sleep();
    }

}