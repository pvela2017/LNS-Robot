/*
This scripts initialize the 4 steering motors
The parameters are:

*/

#include "driving_motors.hpp"

DrivingMotors::DrivingMotors(ros::NodeHandle n)
{
    this->n_ = n;
    this->alarm_clear_ = this->n_.subscribe("/driving_motors/alarm_monitor/clear_alarm", 1, &DrivingMotors::clearAlarmCB, this);
    this->motor_command_ = this->n_.subscribe("/driving_motors/commands", 1, &DrivingMotors::commandsCB, this);
    this->alarm_monitor_ = this->n_.advertise<std_msgs::Int8>("/driving_motors/alarm_monitor/status", 1);
    this->rpm_feedback_ = this->n_.advertise<std_msgs::Int64MultiArray>("/driving_motors/feedback/rpm", 1);
    


    // Buffer initialization
    this->buffer_out_.DLC = 0x08;
    this->buffer_out_.NC1 = 0x00;
    this->buffer_out_.NC2 = 0x00;
    this->buffer_out_.NC3 = 0x00;
    this->buffer_out_.ID = 0x00;
    this->buffer_out_.PID = 0x00;
    this->buffer_out_.D1 = 0x00;
    this->buffer_out_.D2 = 0x00;
    this->buffer_out_.D3 = 0x00;
    this->buffer_out_.D4 = 0x00;
    this->buffer_out_.D5 = 0x00;
    this->buffer_out_.D6 = 0x00;
    this->buffer_out_.D7 = 0x00;
}

DrivingMotors::~DrivingMotors()
{
    // Close the socket
    close(client_);
}


int DrivingMotors::connSocket()
{
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

    // Connect to the server
    status_ = connect(client_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
    while (status_ < 0)
    {
        ROS_ERROR("Socket connection to the server failed");
        status_ = connect(client_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_));
    }

    // Connected
    ROS_INFO("Connected to the server successfully");
    return 0;
}


void DrivingMotors::setSpeed(uint8_t motorID, double rpm)
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 130
    buffer_out_.PID = 0x82;

    // Setup motor ID
    buffer_out_.ID = motorID;

    // Setup Speed bytes
    rpm_ = (int)rpm;
    // If rpm are positive
    if (rpm_ >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        buffer_out_.D2 = (rpm_ >> 8) & 0xff; //byte0
        buffer_out_.D1 = rpm_ & 0xff; //byte1
    }

    // If rpm are negative
    else
    {
        int neg_dec;
        neg_dec = 65535 - abs(rpm_); // FFFF- rpm
        // 16 bytes = 8bytes0 8bytes1 
        buffer_out_.D2  = (neg_dec >> 8) & 0xff;
        buffer_out_.D1 = neg_dec & 0xff;
    }

    // Data Marshalling
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_, 13, 0);

}

int DrivingMotors::alarmMonitor(uint8_t motorID)
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_out_.PID = 0x2B;

    // Setup motor ID
    buffer_out_.ID = motorID;

    // Data Marshalling
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_, 13, 0);

    // Read reply
    if (recv(client_, buffer_in_, 13, MSG_DONTWAIT) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
    {
        ROS_WARN("Cannot read alarms status");
        return -1;
    }

    // Checks for fault if == 0  No fault if == 1 faulty
    if (buffer_in_[6] == 0)
    {
        ROS_DEBUG("Status ok");
        alarm_status_.data = 0;
        alarm_monitor_.publish(alarm_status_);
    }
    else
    {
        ROS_FATAL("MOTOR %d faulty", buffer_out_.ID);
        for (int i = 7; i < 13; i++) // From byte D2 error according to datasheet
        {
            if (buffer_in_[i] == 1)
            {
                alarm_status_.data = i;
                alarm_monitor_.publish(alarm_status_);
                break;
            }
        }
    }

    return 0;
}

int DrivingMotors::rpmFeedback()
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_out_.PID = 0x2B;

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_out_.ID = this->motorID[i];

        // Data Marshalling
        DrivingMotors::Parser();

        // Send command
        send(client_, bytes_, 13, 0);

        // Read reply
        if (recv(client_, buffer_in_, 13, MSG_DONTWAIT) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
        {
            ROS_WARN("Cannot read rpm motor %d", this->motorID[i]);
        }

        rpms_.data[i] = DrivingMotors::byteTorpm(buffer_in_[8], buffer_in_[8]);
    }
    

    return 0;
}

int DrivingMotors::byteTorpm(byte0, byte1)
{
    // Transform the bytes received into the motor rpm
    int motor_rpm, pre_motor_rpm;
    conc_number = byte1 + byte0;
    int dec = int(conc_number, 16);
  
    // Positive rpm
    if (dec <= 32767)
    {
        motor_rpm = dec;
    }  
        
    // Negative rpm
    // http://www.technosoft.ro/KB/index.php?/article/AA-15440/0/Negative-numbers-representation-in-hex.html
    else
    {
        pre_motor_rpm = 65535 - dec; //FFFF - dec
        motor_rpm = -1*pre_motor_rpm;
    }
        
    return motor_rpm;
}

void DrivingMotors::clearAlarmCB(const std_msgs::Int8::ConstPtr& msg)
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_out_.PID = 0x0C;

    // Setup motor ID
    buffer_out_.ID = msg->data;

    // Data Marshalling
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_, 13, 0);
    ROS_INFO("MOTOR %d alarms cleared", buffer_out_.ID);
}

void DrivingMotors::commandsCB(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    // Set motors rpm
    for (int i = 1; i < 5; i++)
    {
        DrivingMotors::setSpeed(this->motorID[i], msg->data[i-1]);
    }
}

void DrivingMotors::Parser()
{
    bytes_[0] = buffer_out_.DLC;
    bytes_[1] = buffer_out_.NC1;
    bytes_[2] = buffer_out_.NC2;
    bytes_[3] = buffer_out_.NC3;
    bytes_[4] = buffer_out_.ID;
    bytes_[5] = buffer_out_.PID;
    bytes_[6] = buffer_out_.D1;
    bytes_[7] = buffer_out_.D2;
    bytes_[8] = buffer_out_.D3;
    bytes_[9] = buffer_out_.D4;
    bytes_[10] = buffer_out_.D5;
    bytes_[11] = buffer_out_.D6;
    bytes_[12] = buffer_out_.D7;
}

void DrivingMotors::clearBuffer()
{
    this->buffer_out_.DLC = 0x08;
    this->buffer_out_.NC1 = 0x00;
    this->buffer_out_.NC2 = 0x00;
    this->buffer_out_.NC3 = 0x00;
    this->buffer_out_.ID = 0x00;
    this->buffer_out_.PID = 0x00;
    this->buffer_out_.D1 = 0x00;
    this->buffer_out_.D2 = 0x00;
    this->buffer_out_.D3 = 0x00;
    this->buffer_out_.D4 = 0x00;
    this->buffer_out_.D5 = 0x00;
    this->buffer_out_.D6 = 0x00;
    this->buffer_out_.D7 = 0x00;
}