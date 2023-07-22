/*
Class to setup  RPMs of driving motors through USB-CAN converter
and read motor and wheel rpms of each motor. It depends on the ros
canopen package.


Subscribe to: /driving_pid/pid/motor1/control_effort
              /driving_pid/pid/motor2/control_effort
              /driving_pid/pid/motor3/control_effort
              /driving_pid/pid/motor4/control_effort

              /received_messages


Publish to: /driving_pid/pid/motor1/state
            /driving_pid/pid/motor2/state
            /driving_pid/pid/motor3/state
            /driving_pid/pid/motor4/state
            
            /sent_messages

by Pablo
Last review: 2023/07/10

TODO: Add function to read alarms
      Add function to clear alarms.

*/


#include "driving_motors.hpp"

DrivingMotors::DrivingMotors(ros::NodeHandle n) : spinner_(6)
{
    /*
    Constructor,
        Initialize spinners, subscribers, publishers
    */


    this->n_ = n;
    spinner_.start();
    this->motor1_command_ = this->n_.subscribe("/driving_pid/pid/motor1/control_effort", 1, &DrivingMotors::m1spCB, this);
    this->motor2_command_ = this->n_.subscribe("/driving_pid/pid/motor2/control_effort", 1, &DrivingMotors::m2spCB, this);
    this->motor3_command_ = this->n_.subscribe("/driving_pid/pid/motor3/control_effort", 1, &DrivingMotors::m3spCB, this);
    this->motor4_command_ = this->n_.subscribe("/driving_pid/pid/motor4/control_effort", 1, &DrivingMotors::m4spCB, this);
    this->receiveCAN_ = this->n_.subscribe<can_msgs::Frame>("/received_messages", 10, &DrivingMotors::canCB, this);

    this->sendtoCAN_ = this->n_.advertise<can_msgs::Frame>("/sent_messages", 10);
    this->motor1_state_ = this->n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor1/state", 1);
    this->motor2_state_ = this->n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor2/state", 1);
    this->motor3_state_ = this->n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor3/state", 1);
    this->motor4_state_ = this->n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor4/state", 1);
    
}

DrivingMotors::~DrivingMotors()
{
    /*
        Destructor,
        Kill spinners.
    */

    spinner_.stop();
}



void DrivingMotors::setSpeed()
{
    /*
        Calculates motor rpm based on the wheel rpm
        and gear box. Then publish the values to the
        CAN-USB interface.
    */

    std::vector<uint8_t> motor_rpm_bytes(2);
    double motor_rpm;

    // Set message
    message_.is_rtr = false;
    message_.is_extended = false;
    message_.is_error = false;
    message_.dlc = 8;


    // Set motor 1
    message_.id = 0x01;
    
    // Set PID 130
    message_.data[0] = 0x86;
    // Set Speed
    motor_rpm = radsecTorpm(wheel_rad_sp_[0])*30.0; // 30 => Gear box 
    motor_rpm_bytes = rpmTobyte(motor_rpm);
    message_.data[1] = motor_rpm_bytes[0];
    message_.data[2] = motor_rpm_bytes[1];
    // Dont care
    for (int i = 3; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Publish message motor 1
    sendtoCAN_.publish(message_);


    // Set motor 2
    message_.id = 0x02;
    
    // Set PID 130
    message_.data[0] = 0x86;
    // Set Speed
    motor_rpm = radsecTorpm(wheel_rad_sp_[1])*30.0; // 30 => Gear box
    motor_rpm_bytes = rpmTobyte(motor_rpm);
    message_.data[1] = motor_rpm_bytes[0];
    message_.data[2] = motor_rpm_bytes[1];
    // Dont care
    for (int i = 3; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Publish message motor 2
    sendtoCAN_.publish(message_);


    // Set motor 3
    message_.id = 0x03;
    
    // Set PID 130
    message_.data[0] = 0x86;
    // Set Speed
    motor_rpm = radsecTorpm(wheel_rad_sp_[2])*30.0; // 30 => Gear box
    motor_rpm_bytes = rpmTobyte(motor_rpm);
    message_.data[1] = motor_rpm_bytes[0];
    message_.data[2] = motor_rpm_bytes[1];
    // Dont care
    for (int i = 3; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Publish message motor 3
    sendtoCAN_.publish(message_);


    // Set motor 4
    message_.id = 0x04;
    
    // Set PID 130
    message_.data[0] = 0x86;
    // Set Speed
    motor_rpm = radsecTorpm(wheel_rad_sp_[3])*30.0; // 30 => Gear box
    motor_rpm_bytes = rpmTobyte(motor_rpm);
    message_.data[1] = motor_rpm_bytes[0];
    message_.data[2] = motor_rpm_bytes[1];
    // Dont care
    for (int i = 3; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Publish message motor 4
    sendtoCAN_.publish(message_);
}



void DrivingMotors::requestFeedback()
{
    /*
        Request the motor rpm of all
        driving motors
    */

    // Set message
    message_.is_rtr = false;
    message_.is_extended = false;
    message_.is_error = false;
    message_.dlc = 8;
  
    // Command PID 138
    message_.data[0] = 0x04;
    message_.data[1] = 0x8A;
    // Dont care
    for (int i = 2; i < 8; i++)
    {
        message_.data[i] = 0;
    }

    // Request rpm
    for (int i = 1; i < 5; i++)
    {
        message_.id = i;
        sendtoCAN_.publish(message_);
    }
}


void DrivingMotors::publishFeedback()
{
    /*
        Calculates the wheel rpms for each motor
        taking in consideration the gear box and
        publish the values
    */

    int motor_rpm;
    double wheel_radsec;
    std_msgs::Float64 wheel_state;

    // Motor 1
    motor_rpm = byteTorpm(wheel_byte1_state_[0], wheel_byte2_state_[0]);
    wheel_radsec = rpmToradsec((double(motor_rpm)/30.0)); // 30 => Gear box
    wheel_state.data = wheel_radsec;
    motor1_state_.publish(wheel_state);

    // Motor 2
    motor_rpm = byteTorpm(wheel_byte1_state_[1], wheel_byte2_state_[1]);
    wheel_radsec = rpmToradsec((double(motor_rpm)/30.0)); // 30 => Gear box
    wheel_state.data = wheel_radsec;
    motor2_state_.publish(wheel_state);

    // Motor 3
    motor_rpm = byteTorpm(wheel_byte1_state_[2], wheel_byte2_state_[2]);
    wheel_radsec = rpmToradsec((double(motor_rpm)/30.0)); // 30 => Gear box
    wheel_state.data = wheel_radsec;
    motor3_state_.publish(wheel_state);

    // Motor 4
    motor_rpm = byteTorpm(wheel_byte1_state_[3], wheel_byte2_state_[3]);
    wheel_radsec = rpmToradsec((double(motor_rpm)/30.0)); // 30 => Gear box
    wheel_state.data = wheel_radsec;
    motor4_state_.publish(wheel_state);
}




void DrivingMotors::m1spCB(const std_msgs::Float64::ConstPtr& msg)
{
    /*
        Save motor 1 speed setpoint (rad/sec)
    */

    wheel_rad_sp_[0] = msg->data;
}

void DrivingMotors::m2spCB(const std_msgs::Float64::ConstPtr& msg)
{
    /*
        Save motor 2 speed setpoint (rad/sec)
    */

    wheel_rad_sp_[1] = msg->data;
}

void DrivingMotors::m3spCB(const std_msgs::Float64::ConstPtr& msg)
{
    /*
        Save motor 3 speed setpoint (rad/sec)
    */

    wheel_rad_sp_[2] = msg->data;
}

void DrivingMotors::m4spCB(const std_msgs::Float64::ConstPtr& msg)
{
    /*
        Save motor 4 speed setpoint (rad/sec)
    */

    wheel_rad_sp_[3] = msg->data;
}

void DrivingMotors::canCB(const can_msgs::Frame::ConstPtr& msg)
{
    /*
        Process all the replies from the driving motor controllers
    */

    switch(msg->id) 
    {
        case 1793:
            if (msg->data[0] == 0x8A) // motor 1 rpm
            {
                wheel_byte1_state_[0] = msg->data[1];
                wheel_byte2_state_[0] = msg->data[2];
            }
            break;

        case 1794:
            if (msg->data[0] == 0x8A) // motor 2 rpm
            {
                wheel_byte1_state_[1] = msg->data[1];
                wheel_byte2_state_[1] = msg->data[2];
            }
            break;

        case 1795:
            if (msg->data[0] == 0x8A) // motor 3 rpm
            {
                wheel_byte1_state_[2] = msg->data[1];
                wheel_byte2_state_[2] = msg->data[2];
            }
            break;

        case 1796:
            if (msg->data[0] == 0x8A) // motor 4 rpm
            {
                wheel_byte1_state_[3] = msg->data[1];
                wheel_byte2_state_[3] = msg->data[2];
            }
            break;

        default:
            break;
    }

}


std::vector<uint8_t> DrivingMotors::rpmTobyte(double rpm)
{
    /*
        Transform rpm into bytes
    */

    std::vector<uint8_t> bytes(2);
    int rpm_int; 

    // Cast to int
    rpm_int = int(rpm);

    // If rpm are positive
    if (rpm_int >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        bytes[1] = (rpm_int >> 8) & 0xff; //byte0
        bytes[0] = rpm_int & 0xff; //byte1
    }

    // If rpm are negative
    else
    {
        int neg_dec;
        neg_dec = 65535 - abs(rpm_int); // FFFF- rpm
        // 16 bytes = 8bytes0 8bytes1 
        bytes[1] = (neg_dec >> 8) & 0xff;
        bytes[0] = neg_dec & 0xff;
    }

    return bytes;
}


int DrivingMotors::byteTorpm(uint8_t byte0, uint8_t byte1)
{
    /*
        Transform the bytes received into the motor rpm
    */

    int rpm, pre_rpm;
    int dec = static_cast<int>(byte1 << 8 | byte0);

    // Positive rpm
    if (dec <= 32767)
    {
        rpm = dec;
    }  
        
    // Negative rpm
    // http://www.technosoft.ro/KB/index.php?/article/AA-15440/0/Negative-numbers-representation-in-hex.html
    else
    {
        pre_rpm = 65535 - dec; //FFFF - dec
        rpm = -1*pre_rpm;
    }
        
    return rpm;
}


double DrivingMotors::rpmToradsec(int rpm)
{
    /* 
        Transform from rpm to rad/s
    */

    double radsec;
    radsec = rpm*((2.0*M_PI)/60.0);  
    return radsec;
}


double DrivingMotors::radsecTorpm(double radsec)
{
    /* 
        Transform from rad/s to rpm 
    */

    double rpm;
    rpm = radsec*(60.0/(2.0*M_PI));  
    return rpm;
}
