/*
Class to setup  RPMs of steering motors through TCP-CAN converter,
check rpm of each motor.
Also check controller alarm status and can clear the alarms.

Connect to socket 2

node: /steering_motors

Subscribe to: /steering_motors/commands
              /steering_motors/alarm_monitor/clear_alarm

Publish to: /steering_motors/alarm_monitor/status
            /steering_motors/feedback/rpm

by Pablo
Last review: 2023/03/30

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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <can_msgs/Frame.h>



class SteeringMotors
{
private:
	ros::NodeHandle n_;
	ros::AsyncSpinner spinner_;
	ros::Subscriber receiveCAN_;
	ros::Subscriber rad_feedback_;
	ros::Publisher sendtoCAN_;
	
	// PID
  // Subscribers PID
  ros::Subscriber motor5_command_;
	ros::Subscriber motor6_command_;
	ros::Subscriber motor7_command_;
	ros::Subscriber motor8_command_;


	// Calibration parameters
	int motor_offsets_[4]  = {0, 0, 0, 0};
	double motor_angles_[4];

	int max_limit_pos_[4] = {297, 295, 300, 290};         // pi/2
	int min_limit_pos_[4] = {-290, -292, -290, -295};     // -pi/2

	// Internals
	can_msgs::Frame message_;
	double steering_angle_sp_[4];
	uint32_t motorID_[5] = {0xFE, 0x05, 0x06, 0x07, 0x08};
	void setPos_calibration(uint32_t, double);
	int checkLimit(int, int, double);
	std::vector<uint8_t> posTobyte(int);


	// Callbacks
	void radFeedbackCB(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void motor5CB(const std_msgs::Float64::ConstPtr&);
  void motor6CB(const std_msgs::Float64::ConstPtr&);
  void motor7CB(const std_msgs::Float64::ConstPtr&);
  void motor8CB(const std_msgs::Float64::ConstPtr&);

	
public:
	SteeringMotors(ros::NodeHandle);
	~SteeringMotors();
	void setPos();
	void calibrationRoutine();
	


};