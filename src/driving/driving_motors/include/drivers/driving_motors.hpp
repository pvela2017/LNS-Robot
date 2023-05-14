/*
Class to setup  RPMs of driving motors through TCP-CAN converter,
check rpm of each motor and speed based on the wheel diameter.
Also check controller alarm status and can clear the alarms.

Connect to socket 1

node: /driving_motors

Subscribe to: /driving_motors/commands
              /driving_motors/alarm_monitor/clear_alarm

Publish to: /driving_motors/alarm_monitor/status
            /driving_motors/feedback/rpm
            /driving_motors/feedback/speed

by Pablo
Last review: 2023/03/23

TODO: 
add multithreading according to
https://codereview.stackexchange.com/questions/151044/socket-client-in-c-using-threads

TODO:
Message get mixed, from feedback() function and alarmMonitor() function
[ WARN] [1679557677.097555829]: Wrong motor alarm reply received 138
[ WARN] [1679557677.097981996]: Wrong motor rpm reply received
[ WARN] [1679557678.097529224]: Wrong motor alarm reply received 138
[ WARN] [1679557678.097844338]: Wrong motor rpm reply received

Also message received order is not always 1,2,3,4 sometimes is random

Possible causes:
- CAN bus is busy transmitting commands from pc to devices, so then they 
  replied at the same time messing up the order

- Ethernet CAN module mess up the order. 
  Unable to debug with logic analyzer cables quality is not good so it cant sample

Workaround:
 Use only feedback() function

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