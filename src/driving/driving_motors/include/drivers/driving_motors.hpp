/*
Class to setup  RPMs of driving motors through USB-CAN converter
and read motor and wheel rpms of each motor. It depends on the ros
canopen package.


Subscribe to: /driving_pid/pid/motor1/control_effort
              /driving_pid/pid/motor2/control_effort
              /driving_pid/pid/motor3/control_effort
              /driving_pid/pid/motor4/control_effort

              /received_messages


Publish to: /driving_pid/pid/motor1/state
            /driving_pid/pid/motor2/state
            /driving_pid/pid/motor3/state
            /driving_pid/pid/motor4/state
            
            /sent_messages

by Pablo
Last review: 2023/07/10

TODO: Add function to read alarms
      Add function to clear alarms.

*/


#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <can_msgs/Frame.h>
#include <string.h>
#include <math.h>
#include <vector>



class DrivingMotors
{
private:
	ros::NodeHandle n_;
	ros::Subscriber motor1_command_;
	ros::Subscriber motor2_command_;
	ros::Subscriber motor3_command_;
	ros::Subscriber motor4_command_;
	ros::Subscriber receiveCAN_;
	ros::Publisher motor1_state_;
	ros::Publisher motor2_state_;
	ros::Publisher motor3_state_;
	ros::Publisher motor4_state_;
	ros::Publisher sendtoCAN_;
	ros::AsyncSpinner spinner_;

	// Internals
	can_msgs::Frame message_;
	std_msgs::Float64MultiArray radsec_;
	double wheel_rad_sp_[4];
	uint8_t wheel_byte1_state_[4];
	uint8_t wheel_byte2_state_[4];

	std::vector<uint8_t> rpmTobyte(double);
	int byteTorpm(uint8_t, uint8_t);
	double rpmToradsec(int);
	double radsecTorpm(double);


	// Callbacks
	void m1spCB(const std_msgs::Float64::ConstPtr&);
	void m2spCB(const std_msgs::Float64::ConstPtr&);
	void m3spCB(const std_msgs::Float64::ConstPtr&);
	void m4spCB(const std_msgs::Float64::ConstPtr&);
	void canCB(const can_msgs::Frame::ConstPtr&);


public:
	DrivingMotors(ros::NodeHandle n);
	~DrivingMotors();
	void setSpeed();
	void requestFeedback();
	void publishFeedback();

};