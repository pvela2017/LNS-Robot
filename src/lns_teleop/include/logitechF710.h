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











