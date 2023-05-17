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


#include "steering_motors.hpp"

SteeringMotors::SteeringMotors(ros::NodeHandle n) : spinner_(6)
{
    /*
    Class Inicialization
    Assign the subscribers and publishers
    Assign Nodehandlers for the multithread queue & callbacks
    */

    this->n_ = n;
    spinner_.start();

    this->sendtoCAN_ = this->n_.advertise<can_msgs::Frame>("/sent_messages", 10);

    this->rad_feedback_ = this->n_.subscribe("/steering_motors/feedback/rad", 1, &SteeringMotors::radFeedbackCB, this); 
    this->motor5_command_ = this->n_.subscribe("/steering_motors/pid/motor5/control_effort", 1, &SteeringMotors::motor5CB, this);
    this->motor6_command_ = this->n_.subscribe("/steering_motors/pid/motor6/control_effort", 1, &SteeringMotors::motor6CB, this);
    this->motor7_command_ = this->n_.subscribe("/steering_motors/pid/motor7/control_effort", 1, &SteeringMotors::motor7CB, this);
    this->motor8_command_ = this->n_.subscribe("/steering_motors/pid/motor8/control_effort", 1, &SteeringMotors::motor8CB, this);

}

SteeringMotors::~SteeringMotors()
{
    spinner_.stop();
}


void SteeringMotors::setPos()
{
    /*
    Transform rpm to byte and send the speed command
    If the DLC is change from 0x08 feedback becomes unstable
    */

    int hard_top, hard_low, pos;
    std::vector<uint8_t> motor_bytes(4);

    // Set message
    message_.is_rtr = false;
    message_.is_extended = false;
    message_.is_error = false;
    message_.dlc = 8;

    
    // Set motor 5
    message_.id = 0x05;

    // Setup Command PID 219
    message_.data[0] = 0xDB;

    // Hard limits
    hard_top = max_limit_pos_[0] + motor_offsets_[0];
    hard_low = min_limit_pos_[0] + motor_offsets_[0];
    pos = SteeringMotors::checkLimit(hard_low, hard_top, steering_angle_sp_[0]);

    // Setup position bytes
    motor_bytes = SteeringMotors::posTobyte(pos);
    for (int i = 0; i < 4; i++)
    {
        message_.data[i+1] = motor_bytes[i];
    }

    for (int i = 5; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Send command
    sendtoCAN_.publish(message_);


    // Set motor 6
    message_.id = 0x06;

    // Setup Command PID 219
    message_.data[0] = 0xDB;

    // Hard limits
    hard_top = max_limit_pos_[1] + motor_offsets_[1];
    hard_low = min_limit_pos_[1] + motor_offsets_[1];
    pos = SteeringMotors::checkLimit(hard_low, hard_top, steering_angle_sp_[1]);

    // Setup position bytes
    motor_bytes = SteeringMotors::posTobyte(pos);
    for (int i = 0; i < 4; i++)
    {
        message_.data[i+1] = motor_bytes[i];
    }

    for (int i = 5; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Send command
    sendtoCAN_.publish(message_);
    

    
    // Set motor 7
    message_.id = 0x07;

    // Setup Command PID 219
    message_.data[0] = 0xDB;

    // Hard limits
    hard_top = max_limit_pos_[2] + motor_offsets_[2];
    hard_low = min_limit_pos_[2] + motor_offsets_[2];
    pos = SteeringMotors::checkLimit(hard_low, hard_top, steering_angle_sp_[2]);

    // Setup position bytes
    motor_bytes = SteeringMotors::posTobyte(pos);
    for (int i = 0; i < 4; i++)
    {
        message_.data[i+1] = motor_bytes[i];
    }

    for (int i = 5; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Send command
    sendtoCAN_.publish(message_);


    // Set motor 8
    message_.id = 0x08;

    // Setup Command PID 219
    message_.data[0] = 0xDB;

    // Hard limits
    hard_top = max_limit_pos_[3] + motor_offsets_[3];
    hard_low = min_limit_pos_[3] + motor_offsets_[3];
    pos = SteeringMotors::checkLimit(hard_low, hard_top, steering_angle_sp_[3]);

    // Setup position bytes
    motor_bytes = SteeringMotors::posTobyte(pos);
    for (int i = 0; i < 4; i++)
    {
        message_.data[i+1] = motor_bytes[i];
    }

    for (int i = 5; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Send command
    sendtoCAN_.publish(message_);
}


void SteeringMotors::calibrationRoutine()
{
    /*
    The zero position will be given by the starting position 
    of the wheel when the motors are powered, thus this routine
    finds the offset for each motor.
    */

    ROS_INFO("Starting Calibration");

    // Alignement flag
    bool align = false;
    // Deadband
    double deadband_rad = 0.00174; // 0.1 degree
    // Offset
    int j = 0;
    // Delay to read the sensor
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    for (int i = 1; i < 5; i++) //TEST cambiar a 1 a 5!
    {
        while (!align)
        {
            ros::spinOnce();
            if (fabs(motor_angles_[i-1]) <= deadband_rad)
            {
                align = true;
                break;
            }

            if(motor_angles_[i-1] > 0)
            {
                j--;
                SteeringMotors::setPos_calibration(motorID_[i], j);
            }
            else if (motor_angles_[i-1] < 0)
            {
                j++;
                SteeringMotors::setPos_calibration(motorID_[i], j);
            }
            ros::Duration(0.2).sleep(); // 0.1 second to let the motor steer
        }
        // Save offsets
        motor_offsets_[i-1] = j;

        // Reset values
        align = false;
        j = 0;
    }

    ROS_INFO("Calibration Successed");

    // Unsubscribe from the topic
    rad_feedback_.shutdown();
}

void SteeringMotors::setPos_calibration(uint32_t motorID, double posCal)
{
    /*
    Transform rpm to byte and send the speed command
    If the DLC is change from 0x08 feedback becomes unstable
    */

    int hard_top, hard_low, pos;
    std::vector<uint8_t> motor_bytes(4);

    // Set message
    message_.is_rtr = false;
    message_.is_extended = false;
    message_.is_error = false;
    message_.dlc = 8;

    
    // Set motor 5
    message_.id = motorID;

    // Setup Command PID 219
    message_.data[0] = 0xDB;

    // Hard limits
    hard_top = max_limit_pos_[motorID-5] + motor_offsets_[motorID-5];
    hard_low = min_limit_pos_[motorID-5] + motor_offsets_[motorID-5];
    pos = SteeringMotors::checkLimit(hard_low, hard_top, posCal);

    // Setup position bytes
    motor_bytes = SteeringMotors::posTobyte(pos);
    for (int i = 0; i < 4; i++)
    {
        message_.data[i+1] = motor_bytes[i];
    }

    for (int i = 5; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Send command
    sendtoCAN_.publish(message_);
}


int SteeringMotors::checkLimit(int hard_low, int hard_top, double pos)
{
    int posint;
    posint = int(pos);
    if (posint > hard_top)
    {
        posint = hard_top;
    }
    if (posint < hard_low)
    {
        posint = hard_low;
    }

    return posint;
}


std::vector<uint8_t> SteeringMotors::posTobyte(int pos)
{
    std::vector<uint8_t> bytes(4);

    // If pos are positive
    if (pos >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        bytes[3] = (pos >> 24) & 0xff;
        bytes[2] = (pos >> 16) & 0xff;
        bytes[1] = (pos >> 8) & 0xff; 
        bytes[0] = pos & 0xff; 
    }

    // If pos are negative
    else
    {
        int neg_dec;
        neg_dec = 4294967295 - abs(pos); // 6 x FF- pos
        // 16 bytes = 8bytes0 8bytes1 
        bytes[3] = (neg_dec >> 24) & 0xff;
        bytes[2] = (neg_dec >> 16) & 0xff;
        bytes[1] = (neg_dec >> 8) & 0xff; 
        bytes[0] = neg_dec & 0xff;
    }

    return bytes;
}



void SteeringMotors::radFeedbackCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    for (int i = 0; i < 4; i++)
    {
        motor_angles_[i] = msg->data[i];
    }    
}

void SteeringMotors::motor5CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 5
    steering_angle_sp_[0] = msg->data;
}

void SteeringMotors::motor6CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 6
    steering_angle_sp_[1] = msg->data;
}

void SteeringMotors::motor7CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 7
    steering_angle_sp_[2] = msg->data;
}

void SteeringMotors::motor8CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 8
    steering_angle_sp_[3] = msg->data;
}
