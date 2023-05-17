/*
Setup driving motors in speed mode.

Connect to socket 1

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
	    message_.data[1] = 0x01;
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
	        case 1793: // motor 1 
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[0] = true;
	            }
	            break;

	        case 1794: // motor 2
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[1] = true;
	            }
	            break;

	        case 1795: // motor 3
	            if (msg->data[0] == 7 && msg->data[1] == 183) // confirm B7 command
	            {
	                success_[2] = true;
	            }
	            break;

	        case 1796: // motor 4
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
    ros::init(argc, argv, "driving_setup");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    setupMotor drivingMotors(n);


    while (!drivingMotors.success_[0])
    {
    	drivingMotors.setMotor(0x01);      
    }
    ROS_INFO("Motor 1 OK");



    while (!drivingMotors.success_[1])
    {
    	drivingMotors.setMotor(0x02);      
    }
    ROS_INFO("Motor 2 OK");



    while (!drivingMotors.success_[2])
    {
    	drivingMotors.setMotor(0x03);      
    }
    ROS_INFO("Motor 3 OK");



    while (!drivingMotors.success_[3])
    {
    	drivingMotors.setMotor(0x04);      
    }
    ROS_INFO("Motor 4 OK");

    
    spinner.stop();
}