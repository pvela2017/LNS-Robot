#include <ros/ros.h>
#include <logitechF710.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "F710_joystick");
  LogitechF710 robot;

  ros::spin();
}