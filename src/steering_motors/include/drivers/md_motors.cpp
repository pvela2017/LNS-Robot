/*
Class to setup  RPMs of driving motors through TCP-CAN converter,
check rpm of each motor and speed based on the wheel diameter.
Also check controller alarm status and can clear the alarms.

Connect to socket 1

node: /driving_motors

Subscribe to: /driving_motors/commands
              /driving_motors/alarm_monitor/clear_alarm

Publish to: /driving_motors/alarm_monitor/status
            /driving_motors/feedback/rpm
            /driving_motors/feedback/speed

by Pablo
Last review: 2023/03/23

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


#include "md_motors.hpp"

MdMotors::MdMotors(ros::NodeHandle n)
{
    this->n_ = n;
    this->alarm_clear_ = this->n_.subscribe("/driving_motors/alarm_monitor/clear_alarm", 1, &MdMotors::clearAlarmCB, this);
    this->motor_command_ = this->n_.subscribe("/driving_motors/commands", 1, &MdMotors::commandsCB, this);
    this->alarm_monitor_ = this->n_.advertise<std_msgs::Int8MultiArray>("/driving_motors/alarm_monitor/status", 1);
    this->rpm_feedback_ = this->n_.advertise<std_msgs::Int64MultiArray>("/driving_motors/feedback/rpm", 1);
    this->speed_feedback_ = this->n_.advertise<std_msgs::Float64MultiArray>("/driving_motors/feedback/speed", 1);
    


    // Buffer initialization
    this->buffer_.DLC = 0x08;
    this->buffer_.NC1 = 0x00;
    this->buffer_.NC2 = 0x00;
    this->buffer_.NC3 = 0x00;
    this->buffer_.ID = 0x00;
    this->buffer_.PID = 0x00;
    this->buffer_.D1 = 0x00;
    this->buffer_.D2 = 0x00;
    this->buffer_.D3 = 0x00;
    this->buffer_.D4 = 0x00;
    this->buffer_.D5 = 0x00;
    this->buffer_.D6 = 0x00;
    this->buffer_.D7 = 0x00;
}

MdMotors::~MdMotors()
{

}


int MdMotors::connSocket()
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


void MdMotors::setSpeed(uint8_t motorID, double rpm)
{
    // Clear the buffer
    MdMotors::clearBuffer();

    // Setup Command PID 130
    buffer_.PID = 0x86;

    // Setup motor ID
    buffer_.ID = motorID;

    // Setup Speed bytes
    rpm_ = (int)rpm;
    // If rpm are positive
    if (rpm_ >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        buffer_.D2 = (rpm_ >> 8) & 0xff; //byte0
        buffer_.D1 = rpm_ & 0xff; //byte1
    }

    // If rpm are negative
    else
    {
        int neg_dec;
        neg_dec = 65535 - abs(rpm_); // FFFF- rpm
        // 16 bytes = 8bytes0 8bytes1 
        buffer_.D2  = (neg_dec >> 8) & 0xff;
        buffer_.D1 = neg_dec & 0xff;
    }

    // Data Marshalling
    MdMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, MSG_DONTWAIT);

}

int MdMotors::alarmMonitor()
{
    // Clear the buffer
    MdMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x04;
    buffer_.D1 = 0x2B;

    // Create vector to store data
    std::vector<int> vec_alarms (4);

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        MdMotors::Parser();

        // Send command
        send(client_, bytes_out_, 13, MSG_DONTWAIT);

        // Read reply
        if (recv(client_, bytes_in_, 13, MSG_WAITALL) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
        {
            ROS_WARN("Cannot read alarms status");
        }

        // Check message received is correct
        if (bytes_in_[5] == buffer_.D1)
        {
            // Checks for fault if == 0  No fault if == 1 faulty
            if (bytes_in_[6] == 0)
            {
                ROS_DEBUG("Status ok");
                alarm_status_.data[i-1] = 0;               
            }
            else
            {
                ROS_FATAL("MOTOR %d faulty", bytes_in_[4]);
                for (int j = 7; j < 13; j++) // From byte D2  to D7 error according to datasheet
                {
                    if (bytes_in_[j] == 1)
                    {
                        // bytes_in_[4] is motor ID 1,2,3,4 so we susbtract -1 to start the array from 0
                        // the postion in the array which motor is faulty. Ex.: vec_alarms[0] -> motor 1
                        vec_alarms[bytes_in_[4] - 1] = j;
                        break;
                    }
                }
            }
        }

        // Received another message
        else
        {
            ROS_WARN("Wrong motor alarm reply received");
        }
    }

    //push data into data blob
    std::vector<int>::const_iterator itr, end(vec_alarms.end());
    for(itr = vec_alarms.begin(); itr!= end; ++itr) 
    {
        alarm_status_.data.push_back(*itr); 
    }

    // Publish topics
    alarm_monitor_.publish(alarm_status_);

    //clear stuff
    vec_alarms.clear();
    alarm_status_.data.clear();

    return 0;
}

int MdMotors::feedback()
{
    // Clear the buffer
    MdMotors::clearBuffer();

    // Setup Command PID 138
    buffer_.PID = 0x04;
    buffer_.D1 = 0x8A;

    // Create vector to store data
    std::vector<int> vec_rpm (4);
    std::vector<float> vec_speed (4);

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        MdMotors::Parser();

        // Send command
        send(client_, bytes_out_, 13, MSG_DONTWAIT);

        // Read reply
        if (recv(client_, bytes_in_, 13, MSG_WAITALL) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
        {
            ROS_WARN("Cannot read rpm motor %d", this->motorID_[i]);
            return -1;
        }
        // Check message received is correct
        if (bytes_in_[5] == buffer_.D1)
        {
            // bytes_in_[4] is motor ID 1,2,3,4 so we susbtract -1 to start the array from 0
            vec_rpm[bytes_in_[4] - 1] = MdMotors::byteTorpm(bytes_in_[6], bytes_in_[7]);
            vec_speed[bytes_in_[4] - 1] = MdMotors::rpmTovel(vec_rpm[bytes_in_[4] - 1]);
        }

        // Received another message
        else
        {
            ROS_WARN("Wrong motor rpm reply received");
            return -1;
        }
    }

    //push data into data blob
    std::vector<int>::const_iterator itr, end(vec_rpm.end());
    for(itr = vec_rpm.begin(); itr!= end; ++itr) 
    {
        rpms_.data.push_back(*itr); 
    }

    std::vector<float>::const_iterator itr2, end2(vec_speed.end());
    for(itr2 = vec_speed.begin(); itr2!= end2; ++itr2) 
    {
        speed_.data.push_back(*itr2); 
    }

    // Publish topics
    rpm_feedback_.publish(rpms_);
    speed_feedback_.publish(speed_);

    //clear stuff
    vec_rpm.clear();
    rpms_.data.clear();
    vec_speed.clear();
    speed_.data.clear();
    
    return 0;
}

int MdMotors::byteTorpm(uint8_t byte0, uint8_t byte1)
{
    // Transform the bytes received into the motor rpm
    int motor_rpm, pre_motor_rpm;
    int dec = static_cast<int>(byte1 << 8 | byte0);

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

float MdMotors::rpmTovel(int motor_rpm)
{
    // Transform from motor rpm to m/s linear velocity
    float speed, real_rpm;
    real_rpm = motor_rpm/50.0; // DRIVING_GEAR_BOX_RATIO 50 
    speed = real_rpm*(2.0*0.4*3.1415)/60.0;  // WHEELS_RADIUS 0.4  
    return speed;
}
    

void MdMotors::clearAlarmCB(const std_msgs::Int8::ConstPtr& msg)
{
    // Clear the buffer
    MdMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x0C;

    // Setup motor ID
    buffer_.ID = msg->data;

    // Data Marshalling
    MdMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, MSG_DONTWAIT);
    ROS_INFO("MOTOR %d alarms cleared", buffer_.ID);
}

void MdMotors::commandsCB(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    // Set motors rpm
    for (int i = 1; i < 5; i++)
    {
        MdMotors::setSpeed(this->motorID_[i], msg->data[i-1]);
    }
}

void MdMotors::emergencyStop()
{
    // Set motors rpm to 0
    for (int i = 1; i < 5; i++)
    {
        MdMotors::setSpeed(this->motorID_[i], 0);
    }

    // Close the socket
    close(client_);
}

void MdMotors::Parser()
{
    bytes_out_[0] = buffer_.DLC;
    bytes_out_[1] = buffer_.NC1;
    bytes_out_[2] = buffer_.NC2;
    bytes_out_[3] = buffer_.NC3;
    bytes_out_[4] = buffer_.ID;
    bytes_out_[5] = buffer_.PID;
    bytes_out_[6] = buffer_.D1;
    bytes_out_[7] = buffer_.D2;
    bytes_out_[8] = buffer_.D3;
    bytes_out_[9] = buffer_.D4;
    bytes_out_[10] = buffer_.D5;
    bytes_out_[11] = buffer_.D6;
    bytes_out_[12] = buffer_.D7;
}

void MdMotors::clearBuffer()
{
    this->buffer_.DLC = 0x08;
    this->buffer_.NC1 = 0x00;
    this->buffer_.NC2 = 0x00;
    this->buffer_.NC3 = 0x00;
    this->buffer_.ID = 0x00;
    this->buffer_.PID = 0x00;
    this->buffer_.D1 = 0x00;
    this->buffer_.D2 = 0x00;
    this->buffer_.D3 = 0x00;
    this->buffer_.D4 = 0x00;
    this->buffer_.D5 = 0x00;
    this->buffer_.D6 = 0x00;
    this->buffer_.D7 = 0x00;
}