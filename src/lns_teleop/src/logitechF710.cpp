#include "logitechF710.h"

LogitechF710::LogitechF710():
  linear_(4),
  angular_(3)
{
	// Send commands to swerve controller
	vel_pub_ = n_.advertise<geometry_msgs::Twist>("/swerve_controller/cmd_vel", 1);
	// Cancel current goal
	cancel_goal_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

	// Subscribe Joystick topic
	joy_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 10, &LogitechF710::joyCallback, this);

	// Define the joystick axis to use
  	n_.param("axis_linear", linear_, linear_);
	n_.param("axis_angular", angular_, angular_);

	// Define the maximum values as joystick goes from 0 -> 1 
	n_.param("scale_angular", a_scale_, a_scale_);
	n_.param("scale_linear", l_scale_, l_scale_);
}

LogitechF710::~LogitechF710()
{

}


void LogitechF710::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// Create message
	geometry_msgs::Twist twist;

	if (joy->buttons[1])
	{
		// EMERGENCY STOP
		actionlib_msgs::GoalID cancel;
		cancel.stamp = ros::Time::now();
		cancel.id = "";
		
		cancel_goal_.publish(cancel);

		twist.angular.z = 0;
		twist.linear.y = 0;
		twist.linear.x = 0;
	}
	else
	{
		// Manual Control
		twist.angular.z = a_scale_*joy->axes[angular_];
		twist.linear.x = l_scale_*joy->axes[linear_];		
	}
	vel_pub_.publish(twist);
}