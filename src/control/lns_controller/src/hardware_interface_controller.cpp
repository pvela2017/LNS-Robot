/*
Main loop to interface the hardware
with ROS control
    Loop Rate: 30 Hz




by Pablo
Last review: 2023/07/10

*/

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <lns.hpp>


int main(int argc, char** argv)
{
    // initialize the hardware interface ros node 
    ros::init(argc, argv, "lns_hardware");
    ros::NodeHandle nh;
    
    // Create a hardware interface object and connect it to the controller manager
    LnsRobot robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);
    
    // Set the period and update rate for 30Hz 
    ros::Duration period(0.0333);
    ros::Rate rate(1.0/period.toSec());
 
    // Create a new spinner thread for the callback method and start it
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Create an infinite loop  for running the hardware interface
    while(ros::ok())
    {
        robot.read();
        cm.update(ros::Time::now(), period);
        robot.write(period);
        rate.sleep();
    }
    spinner.stop();
    return 0;
}