/*
Setup steering motors in position mode.


by Pablo
Last review: 2023/07/10

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
			Setup motor controller in Position Mode
		*/

		// Set message
	    message_.is_rtr = false;
	    message_.is_extended = false;
	    message_.is_error = false;
	    message_.dlc = 8;


	    // Set motor id
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
	    
	    // Set operation mode: position   PID:#183 [Read Write (No need for 0xaa)] 
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
    	/*
        	Process all the replies from the steering motor controllers
    	*/

	    switch(msg->id) 
	    {
	        case 1797: // motor 1 
	            if (msg->data[0] == 7 && msg->data[1] == 183) // Confirm B7 command configurated
	            {
	                success_[0] = true;
	            }
	            break;

	        case 1798: // motor 2
	            if (msg->data[0] == 7 && msg->data[1] == 183) // Confirm B7 command configurated
	            {
	                success_[1] = true;
	            }
	            break;

	        case 1799: // motor 3
	            if (msg->data[0] == 7 && msg->data[1] == 183) // Confirm B7 command configurated
	            {
	                success_[2] = true;
	            }
	            break;

	        case 1800: // motor 4
	            if (msg->data[0] == 7 && msg->data[1] == 183) // Confirm B7 command configurated
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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    setupMotor steeringMotors(n);


    // Check motor 5 setup
    while (!steeringMotors.success_[0])
    {
    	steeringMotors.setMotor(0x05);      
    }
    ROS_INFO("Motor 5 OK");


    // Check motor 6 setup
    while (!steeringMotors.success_[1])
    {
    	steeringMotors.setMotor(0x06);      
    }
    ROS_INFO("Motor 6 OK");


    // Check motor 7 setup
    while (!steeringMotors.success_[2])
    {
    	steeringMotors.setMotor(0x07);      
    }
    ROS_INFO("Motor 7 OK");


    // Check motor 8 setup
    while (!steeringMotors.success_[3])
    {
    	steeringMotors.setMotor(0x08);      
    }
    ROS_INFO("Motor 8 OK");

    spinner.stop();
}