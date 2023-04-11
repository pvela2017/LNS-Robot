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
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <string.h>
#include <vector>
#include <sys/wait.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#define SERVER_IP "192.168.0.7"
#define PORT 20001

struct message
{
	// Message structure according to CAN communication 
	// protocol V12 for MD motor controllers
	uint8_t DLC;
	uint8_t NC1;
	uint8_t NC2;
	uint8_t NC3;
	uint8_t ID;
	// Data Field
	uint8_t PID;
	uint8_t D1;
	uint8_t D2;
	uint8_t D3;
	uint8_t D4;
	uint8_t D5;
	uint8_t D6;
	uint8_t D7;
};

class DrivingMotors
{
private:
	ros::NodeHandle n_;
	ros::Subscriber alarm_clear_;
	ros::Subscriber motor_command_;
	ros::Publisher alarm_monitor_;
	ros::Publisher rpm_feedback_;
	ros::Publisher speed_feedback_;
	ros::Publisher radsec_feedback_;

	// Socket variables
	struct sockaddr_in serv_addr_;
	int client_;
	int status_;

	// Parser
	void Parser();
	void clearBuffer();
	uint8_t bytes_out_[13];
	message buffer_;
	uint8_t bytes_in_[13];
	int rpm_;

	// Internals
	std_msgs::Int8MultiArray alarm_status_;
	std_msgs::Int64MultiArray rpms_;
	std_msgs::Float64MultiArray speed_;
	std_msgs::Float64MultiArray radsec_;
	uint8_t motorID_[5] = {0xFE, 0x01, 0x02, 0x03, 0x04};
	void setSpeed(uint8_t, double);
	int byteTorpm(uint8_t, uint8_t);
	float rpmTovel(int);
	float rpmToradsec(int);

	// Callbacks
	void commandsCB(const std_msgs::Int64MultiArray::ConstPtr&);
	void clearAlarmCB(const std_msgs::Int8::ConstPtr&);

	

public:
	DrivingMotors(ros::NodeHandle n);
	~DrivingMotors();
	void emergencyStop();
	int connSocket();
	int alarmMonitor();
	int feedback();
};