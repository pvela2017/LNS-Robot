#include <ros/ros.h>
#include <kinematics.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    Kinematics controller(n);


    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}
