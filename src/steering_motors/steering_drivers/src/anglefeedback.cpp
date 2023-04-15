
#include <sys/wait.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


#define SERVER_IP "192.168.0.8"
#define PORT 3131


class Socket
{
private:
	// Socket variables
	struct sockaddr_in serv_addr_;
	int client_;
	int status_;

	// Parser
	void clearBuffer();
	uint8_t bytes_out_[13];
	uint8_t bytes_in_[13];

public:
	int Socket::connSocket();
};

int Socket::connSocket()
{
    /*
    Create and connect the client
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
}


int Socket::ReadEncoder()
{
	send(client_, "?0", 2, MSG_WAITALL);

	// Read reply
    if (recv(client_, bytes_in_, 13, MSG_WAITALL) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
    {
        ROS_WARN("Cannot read alarms status");
    }
    else
    {
    	
    }

}

    
