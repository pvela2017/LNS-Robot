/*
This scripts initialize the 4 steering motors
The parameters are:

*/
#include <ros/ros.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


#define SERVER_IP "192.168.0.7"
#define PORT "20001"

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
	ros::Subscriber cmd_vel_sub_;
	// Socket variables
	struct sockaddr_in serv_addr_;
	int client_;
	int status_;
	// Parser
	uint8_t *bytes_[13];
	message buffer_;
	

public:
	DrivingMotors();
	int connSocket();
	uint8_t * parser(message);
	void setSpeed();
	/*
	
	void rpmTobyte();
	void velTorpm();
	void feedback();
	void alarmMonitor();
	void clearAlarms();
	*/

};