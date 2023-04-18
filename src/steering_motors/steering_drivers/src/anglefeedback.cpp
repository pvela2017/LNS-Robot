#include <regex>
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

    // ROS
    ros::NodeHandle n_;
    ros::Publisher angle_feedback_rad_;
    ros::Publisher angle_motor5_rad_;
    ros::Publisher angle_motor6_rad_;
    ros::Publisher angle_motor7_rad_;
    ros::Publisher angle_motor8_rad_;

	// Parser
	uint8_t bytes_in_[13];

    // Variables inutiles placa ql
    char buffer_[18];
    std::string rcv_;
    int bytesReceived_ = 0;


    //
    double angles_[4];
    double offsets_[4] = {0.0, 0.0, -0.01745329238474369, 0.01745329238474369};
    std_msgs::Float64MultiArray feedback_;
    std_msgs::Float64 feedback_motor5_;
    std_msgs::Float64 feedback_motor6_;
    std_msgs::Float64 feedback_motor7_;
    std_msgs::Float64 feedback_motor8_;

    // Methods
    double degTorad(double);
    double angleWrap(double);
    bool isNumber(std::string);

public:
    Socket(ros::NodeHandle);
    ~Socket();
	int connSocket();
    int ReadEncoder();
};

Socket::Socket(ros::NodeHandle n)
{
    this->n_ = n;
    this->angle_feedback_rad_ = this->n_.advertise<std_msgs::Float64MultiArray>("/steering_motors/feedback/rad", 1);
    this->angle_motor5_rad_ = this->n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor5/state", 1);
    this->angle_motor6_rad_ = this->n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor6/state", 1);
    this->angle_motor7_rad_ = this->n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor7/state", 1);
    this->angle_motor8_rad_ = this->n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor8/state", 1);

}

Socket::~Socket()
{
}

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

    /*
	// Read reply
    ESTA PARTE SERIA SI ESTUVIERA FUNCIONANDO MI PLACA
    if (recv(client_, bytes_in_, 15, MSG_WAITALL) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
    {
        ROS_WARN("Cannot read alarms status");
        return -1;
    }
    else
    {
        // Save angles 
    	angles_[0] = angleWrap(degTorad(bytes_in_[0]) - offsets_[0]);
        angles_[1] = angleWrap(degTorad(bytes_in_[1]) - offsets_[0]);
        angles_[2] = angleWrap(degTorad(bytes_in_[2]) - offsets_[0]);
        angles_[3] = angleWrap(degTorad(bytes_in_[3]) - offsets_[0]);
    }*/


    // Read reply

    if (recv(client_, buffer_, 18, MSG_WAITALL) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
    {
        ROS_WARN("Cannot read encoders");
        return -1;

    }
    if (buffer_[0] == '!')
    {
        ROS_WARN("RPI ERROR");
        return -1;
    }
    else
    {
        //ROS_WARN("%c  %c  %c  %c  %c  %c  %c  %c  %c  %c  %c  %c  %c  %c  %c",buffer_[0], buffer_[1], buffer_[2], buffer_[3], buffer_[4], buffer_[5], buffer_[6], buffer_[7], buffer_[8], buffer_[9], buffer_[10], buffer_[11], buffer_[12], buffer_[13], buffer_[14]);
        // Separate the values
        std::string temp;

        temp += buffer_[0];
        temp += buffer_[1];
        temp += buffer_[2];
        //ROS_WARN("%s",temp.c_str());
        if (!Socket::isNumber(temp))
        {
            return -1;
        }
        angles_[0] = double(stoi(temp));
        temp.clear();

        temp += buffer_[4];
        temp += buffer_[5];
        temp += buffer_[6];
        //ROS_WARN("%s",temp.c_str());
        if (!Socket::isNumber(temp))
        {
            return -1;
        }
        angles_[1] = double(stoi(temp));
        temp.clear();

        temp += buffer_[8];
        temp += buffer_[9];
        temp += buffer_[10];
        //ROS_WARN("%s",temp.c_str());
        if (!Socket::isNumber(temp))
        {
            return -1;
        }
        angles_[2] = double(stoi(temp));
        temp.clear();

        temp += buffer_[12];
        temp += buffer_[13];
        temp += buffer_[14];
        //ROS_WARN("%s",temp.c_str());
        if (!Socket::isNumber(temp))
        {
            return -1;
        }
        angles_[3] = double(stoi(temp));
        temp.clear();

        // Save angles 
        angles_[0] = angleWrap(degTorad(angles_[0]) - offsets_[0]);
        angles_[1] = angleWrap(degTorad(angles_[1]) - offsets_[1]);
        angles_[2] = angleWrap(degTorad(angles_[2]) - offsets_[2]);
        angles_[3] = angleWrap(degTorad(angles_[3]) - offsets_[3]);
    }


    /*
    angles_[0] = angleWrap(degTorad(bytes_in_[0]) - offsets_[0]);
    angles_[1] = angleWrap(degTorad(bytes_in_[1]) - offsets_[0]);
    angles_[2] = angleWrap(degTorad(bytes_in_[2]) - offsets_[0]);
    angles_[3] = angleWrap(degTorad(bytes_in_[3]) - offsets_[0]);*/

    //angles_[0] = bytes_in_[0] - offsets_[0];
    //angles_[1] = bytes_in_[1] - offsets_[0];
    //angles_[2] = bytes_in_[2] - offsets_[0];
    //angles_[3] = bytes_in_[3] - offsets_[0];



    feedback_motor5_.data = angles_[0];
    feedback_motor6_.data = angles_[1];
    feedback_motor7_.data = angles_[2];
    feedback_motor8_.data = angles_[3];

    // Create vector to store data
    std::vector<float> vec_rad (4);

    for (int i = 0; i < 4; i++)
    {
        vec_rad[i] = angles_[i];
    }

    std::vector<float>::const_iterator itr, end(vec_rad.end());
    for(itr = vec_rad.begin(); itr!= end; ++itr) 
    {
        feedback_.data.push_back(*itr); 
    }

    // Publish
    angle_feedback_rad_.publish(feedback_);
    angle_motor5_rad_.publish(feedback_motor5_);
    angle_motor6_rad_.publish(feedback_motor6_);
    angle_motor7_rad_.publish(feedback_motor7_);
    angle_motor8_rad_.publish(feedback_motor8_);

    // Clear stuff
    vec_rad.clear();
    feedback_.data.clear();

    return 0;
}

double Socket::degTorad(double degrees)
{
    /*
    Function to transform deg to rad
    */
    double angle_rad;
    angle_rad = degrees*M_PI/180.0;
    return angle_rad;
}
    


double Socket::angleWrap(double radians)
{
    if (radians > M_PI)
    {
        radians -= 2.*M_PI;
    }       
    return radians;
}

bool Socket::isNumber(std::string token )
{
    return std::regex_match( token, std::regex( ( "((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?" ) ) );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_feedback");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);

    Socket steering_angle_feedback(n);
    steering_angle_feedback.connSocket();


    while (ros::ok)
    {
        steering_angle_feedback.ReadEncoder();
        loop_rate.sleep();
    }

}