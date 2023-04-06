/*

TODO:
	Change to real time controller using: 
		- https://github.com/ros-controls/realtime_tools
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64.h>


class Kinematics
{
private:
	ros::NodeHandle n_;
	ros::Subscriber cmd_vel_sub_;
	ros::Publisher driving_rpms_;
	ros::Publisher steering_motor5_angle_;
	ros::Publisher steering_motor6_angle_;
	ros::Publisher steering_motor7_angle_;
	ros::Publisher steering_motor8_angle_;
	geometry_msgs::Twist twist_command_;
	geometry_msgs::Twist last0_cmd_;
	geometry_msgs::Twist last1_cmd_;
	std_msgs::Int64MultiArray rpms_;
    std_msgs::Float64 motor5_angle_;
    std_msgs::Float64 motor6_angle_;
    std_msgs::Float64 motor7_angle_;
    std_msgs::Float64 motor8_angle_;

    // Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width):
    double track_;
    // Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint:
    // We consider that the distance is the same for every wheel
    double wheel_steering_y_offset_;

    // Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    // Wheel base (distance between front and rear wheel):
    double wheel_base_;

    // Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

	void cmdVelCB(const geometry_msgs::Twist&);
	void updateCommand();
	double angleScale(double);
	double speedTorpm(double);
	double angleLimit(double);
	double rpmLimit(double rpm);

public:
	Kinematics(ros::NodeHandle);
	~Kinematics();
	
};