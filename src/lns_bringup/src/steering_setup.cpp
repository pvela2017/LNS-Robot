/*
Setup steering motors in speed mode.

Connect to socket 2

TODO: Use ROS Param to load server ip and port

by Pablo
Last review: 2023/03/30
*/

#include <ros/ros.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>


#define SERVER_IP "192.168.0.7"
#define PORT 20005


class setupMotor
{
private:
	// Socket variables
    struct sockaddr_in serv_addr_;
	int client_;
	int status_;
	
	uint8_t bytes_out_[13] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t bytes_in_[13];
    bool flag_success_ = false;

public:
	setupMotor()
	{};
	~setupMotor()
	{
		// Close the socket
    	close(client_);
	};

	int connectSocket()
	{
		/*
		Create and connect socket
		*/

		// Create the socket 
		if ((client_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    	{
	        ROS_FATAL("Socket creation error");
	        return -1;
	    }

	    serv_addr_.sin_family = AF_INET;
	    serv_addr_.sin_port = htons(PORT);
	    
	    if (inet_pton(AF_INET, SERVER_IP, &serv_addr_.sin_addr) <= 0)
	    {
	        ROS_FATAL("Invalid address");
	        return -1;
	    }

	    // Set the client as non-blocking
	    // On Linux, this command can change only the O_APPEND, O_ASYNC, O_DIRECT, O_NOATIME, and O_NONBLOCK flags.
	    fcntl(client_, F_SETFL, O_NONBLOCK);

	    // Connect to the server
	    status_ = connect(client_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
	    while (status_ < 0)
	    {
	        ROS_ERROR("Socket connection to the server failed");
	        status_ = connect(client_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
	        sleep(1);
	    }

	    // Connected
	    ROS_INFO("Connected to the server successfully");
	    return 0;
	};

	void sendCommand(int i)
	{
		/*
		Send command and check the reply is ok (0x07)
		*/

		flag_success_ = false;
		while (!flag_success_)
    	{
    		send(client_, bytes_out_, 13, MSG_WAITALL);

	    	if (recv(client_, bytes_in_, 13, MSG_WAITALL) > 0) // reply its only 1 byte so total msg lenght 6 bytes (DLC, NC1, NC2, NC3, ID, D1).
	    	{
	    		// Check message received is correct 0x07
	        	if (bytes_in_[5] == 7)
	        	{
	        		flag_success_ = true; // PID: #06 setup correctly
	        	}
	        	else
	        	{
	        		ROS_ERROR("Setup %d Failed", i);
	        	}
	    	}
	    	//ROS_DEBUG("%d", errno);	    	
	    	ros::Duration(1).sleep(); // send command again every 10 ms
	    }    	
	};

	void sendCommandNoCheck()
	{
		/*
		Send command without checking any reply from the motor controller
		*/

		send(client_, bytes_out_, 13, MSG_WAITALL);	    	
	    ros::Duration(1).sleep(); // send command again every 10 ms  	
	};

	void onlyOnce(uint8_t motorId)
	{
		/*
		Setup number of poles, motor direction, zero speed and brakes.
		Only setup once in life!
		*/

		bytes_out_[4] = motorId;

	    // Set number of poles  8 PID:#21 [Read Write (No need for 0xaa)]
	    bytes_out_[5] = 0x15;
	    bytes_out_[6] = 0x01;
	    // Send command
	    setupMotor::sendCommand(1);

	    // Set inversion of moving direction (Some motors) PID:#16 [Read Write (No need for 0xaa)]
	    if (bytes_out_[4] == 0x03 || bytes_out_[4] == 0x04)
	    {
	    	bytes_out_[5] = 0x10;
	    	bytes_out_[6] = 0x01;
	    }
	    else
	    {
	    	bytes_out_[5] = 0x10;
	    	bytes_out_[6] = 0x00;
	    }
	    // Send command
	    setupMotor::sendCommand(2);

	    // Set speed = 0 then control zero speed, not braking  PID:#24 [Read Write (No need for 0xaa)]
	    bytes_out_[5] = 0x18;
	    bytes_out_[6] = 0x01;
	    // Send command
	    setupMotor::sendCommand(3);

	    // Set operation mode: speed   PID:#183 [Read Write (No need for 0xaa)] 
	    bytes_out_[5] = 0xB7;
	    bytes_out_[6] = 0x01;
	    // Send command
	    setupMotor::sendCommand(4);

	   	// Free mode on, brake OFF  PID:#05 [Command]
	    bytes_out_[5] = 0x05;
	    bytes_out_[6] = 0x00;
	    // Send command
	    setupMotor::sendCommand(5);
	};


	void setMotor(uint8_t motorId)
	{
		/*
		Setup motor controller in Speed Mode
		*/

		bytes_out_[4] = motorId;

		// Dummy command, without sending this first the motor doesnt setup speed mode
		// TODO: WHY???
	    bytes_out_[5] = 0x86;
	    bytes_out_[6] = 0x00;
	    // Send command
	    setupMotor::sendCommandNoCheck();

	    // Set operation mode: speed   PID:#183 [Read Write (No need for 0xaa)] 
	    bytes_out_[5] = 0xB7;
	    bytes_out_[6] = 0x01;
	    // Send command
	    setupMotor::sendCommand(1);
	};

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_setup");
    ros::NodeHandle n;
    
    setupMotor drivingMotors;
    drivingMotors.connectSocket();

    // Motor 1
    drivingMotors.setMotor(0x05);
    ROS_INFO("Motor 5 OK");

    // Motor 2
    drivingMotors.setMotor(0x06);
    ROS_INFO("Motor 6 OK");

    // Motor 3
    drivingMotors.setMotor(0x07);
    ROS_INFO("Motor 7 OK");

    // Motor 4
    drivingMotors.setMotor(0x08);
    ROS_INFO("Motor 8 OK");
}