/*
Setup steering motors in speed mode.

Connect to socket 2

TODO: Use ROS Param to load server ip and port

by Pablo
Last review: 2023/03/30
*/

#include <ros/ros.h>
#include <can_msgs/Frame.h>


class setupMotor
{
private:
	ros::NodeHandle n_;
	ros::Subscriber receiveCAN_;
	ros::Publisher sendtoCAN_;

	// Internals
	can_msgs::Frame message_;	

public:
	bool success_[4] = {false, false, false, false};
	setupMotor(ros::NodeHandle n)
	{
		this->n_ = n;
		this->receiveCAN_ = this->n_.subscribe<can_msgs::Frame>("/received_messages", 20, &setupMotor::canCB, this);
    	this->sendtoCAN_ = this->n_.advertise<can_msgs::Frame>("/sent_messages", 10);
	};
	~setupMotor()
	{
	};


	void setMotor(uint32_t idmotor)
	{
		/*
		Setup motor controller in Speed Mode
		*/

		// Set message
	    message_.is_rtr = false;
	    message_.is_extended = false;
	    message_.is_error = false;
	    message_.dlc = 8;


	    // Broadcast
	    message_.id = idmotor;


	    // Dummy
	    message_.data[0] = 0x86;
	    for (int i = 1; i < 8; i++)
    	{
        	message_.data[i] = 0;
    	}
    	ros::Duration(1).sleep();
    	sendtoCAN_.publish(message_);
    	ros::Duration(1).sleep();
	    
	    // Set operation mode: speed   PID:#183 [Read Write (No need for 0xaa)] 
	    message_.data[0] = 0xB7;
	    message_.data[1] = 0x02;
	    for (int i = 2; i < 8; i++)
    	{
        	message_.data[i] = 0;
    	}
    	sendtoCAN_.publish(message_);
    	ros::spinOnce();
	};


	void canCB(const can_msgs::Frame::ConstPtr& msg)
	{
    	/**/
	    switch(msg->id) 
	    {
	        case 1797: // motor 1 
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[0] = true;
	            }
	            break;

	        case 1798: // motor 2
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[1] = true;
	            }
	            break;

	        case 1799: // motor 3
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[2] = true;
	            }
	            break;

	        case 1800: // motor 4
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[3] = true;
	            }
	            break;

	        default:
	            break;
	    }

	};

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_setup");
    ros::NodeHandle n;
    
    setupMotor steeringMotors(n);

    // Motor 1
    steeringMotors.setMotor(0x05);
    ROS_INFO("Motor 5 OK");

    // Motor 2
    steeringMotors.setMotor(0x06);
    ROS_INFO("Motor 6 OK");

    // Motor 3
    steeringMotors.setMotor(0x07);
    ROS_INFO("Motor 7 OK");

    // Motor 4
    steeringMotors.setMotor(0x08);
    ROS_INFO("Motor 8 OK");
}