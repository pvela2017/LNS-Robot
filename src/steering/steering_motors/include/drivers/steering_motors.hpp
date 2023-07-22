/*
Class to setup the angle of the steering motors through USB-CAN 
converter. It depends on the ros canopen package.


Subscribe to: /steering_motors/pid/motor5/control_effort
              /steering_motors/pid/motor6/control_effort
              /steering_motors/pid/motor7/control_effort
              /steering_motors/pid/motor8/control_effort
              /steering_motors/feedback/rad


Publish to: /sent_messages

by Pablo
Last review: 2023/07/10

TODO: Add function to read alarms
      Add function to clear alarms.

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