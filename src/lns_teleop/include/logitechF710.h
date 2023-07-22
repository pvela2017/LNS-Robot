/*
Class to use the Logitech F710 gamepad
to control the robot.


Subscribe to: /joy


Publish to: /swerve_controller/cmd_vel
            /move_base/cancel

by Pablo
Last review: 2023/07/10

TODO: Generate linear Y commands

*/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>


class LogitechF710
{
private:
  // ROS
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Publisher cancel_goal_;
  ros::Subscriber joy_sub_;

  // Define the joystick axis to use
  int linear_, angular_;
  // Define the maximum values as joystick goes from 0 -> 1 
  double l_scale_, a_scale_;

  // Method
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  public:
  LogitechF710();
  ~LogitechF710();

};











