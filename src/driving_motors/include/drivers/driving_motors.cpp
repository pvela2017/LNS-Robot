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
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, 0);

}

// TODO: remove after test
/*
int DrivingMotors::alarmMonitor(uint8_t motorID)
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x2B;

    // Setup motor ID
    buffer_.ID = motorID;

    // Data Marshalling
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, 0);

    // Read reply
    if (recv(client_, bytes_in_, 13, MSG_DONTWAIT) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
    {
        ROS_WARN("Cannot read alarms status");
        return -1;
    }

    // Checks for fault if == 0  No fault if == 1 faulty
    if (bytes_in_[6] == 0)
    {
        ROS_DEBUG("Status ok");
        alarm_status_.data = 0;
        alarm_monitor_.publish(alarm_status_);
    }
    else
    {
        ROS_FATAL("MOTOR %d faulty", buffer_.ID);
        for (int i = 7; i < 13; i++) // From byte D2 error according to datasheet
        {
            if (bytes_in_[i] == 1)
            {
                alarm_status_.data = i;
                alarm_monitor_.publish(alarm_status_);
                break;
            }
        }
    }

    return 0;
}
*/


int DrivingMotors::alarmMonitor()
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x04;
    buffer_.D1 = 0x2B;

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        DrivingMotors::Parser();

        // Send command
        send(client_, bytes_out_, 13, 0);

        // Read reply
        if (recv(client_, bytes_in_, 13, MSG_DONTWAIT) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
        {
            ROS_WARN("Cannot read alarms status");
        }

        // Check message received is correct
        if (bytes_in_[4] == buffer_.ID && bytes_in_[5] == buffer_.PID)
        {
            // Checks for fault if == 0  No fault if == 1 faulty
            if (bytes_in_[6] == 0)
            {
                ROS_DEBUG("Status ok");
                alarm_status_.data[i-1] = 0;               
            }
            else
            {
                ROS_FATAL("MOTOR %d faulty", buffer_.ID);
                for (int j = 7; j < 13; j++) // From byte D2  to D7 error according to datasheet
                {
                    if (bytes_in_[j] == 1)
                    {
                        alarm_status_.data[i-1] = j;
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

    alarm_monitor_.publish(alarm_status_);

    return 0;
}

int DrivingMotors::feedback()
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 138
    buffer_.PID = 0x04;
    buffer_.D1 = 0x8A;

    // Create vector to store data
    std::vector<int> vec_rpm (4);
    std::vector<int> vec_speed (4);

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        DrivingMotors::Parser();

        // Send command
        send(client_, bytes_out_, 13, 0);

        // Read reply
        if (recv(client_, bytes_in_, 13, MSG_DONTWAIT) <= 0) //attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
        {
            ROS_WARN("Cannot read rpm motor %d", this->motorID_[i]);
            return -1;
        }
        // Check message received is correct
        if (bytes_in_[5] == buffer_.D1)
        {
            ROS_WARN("%d", bytes_in_[4]);
            vec_rpm[bytes_in_[4] - 1] = DrivingMotors::byteTorpm(bytes_in_[6], bytes_in_[7]);
            ROS_WARN("Adssad");
            //vec_speed[bytes_in_[4] - 1] = DrivingMotors::rpmTovel(vec_rpm[bytes_in_[4] - 1]);
            //speed_.data[bytes_in_[4] - 1] = DrivingMotors::rpmTovel(rpms_.data[i-1]);
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

    //rpm_feedback_.publish(rpms_);
    //speed_feedback_.publish(speed_);

    //clear stuff
    vec_rpm.clear();
    vec_speed.clear();
    
    return 0;
}

int DrivingMotors::byteTorpm(uint8_t byte0, uint8_t byte1)
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

float DrivingMotors::rpmTovel(int motor_rpm)
{
    // Transform from motor rpm to m/s linear velocity
    float speed, real_rpm;
    real_rpm = motor_rpm/50.0; // DRIVING_GEAR_BOX_RATIO 50 
    speed = real_rpm*(2.0*0.4*3.1415)/60.0;  // WHEELS_RADIUS 0.4  
    return speed;
}
    

void DrivingMotors::clearAlarmCB(const std_msgs::Int8::ConstPtr& msg)
{
    // Clear the buffer
    DrivingMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x0C;

    // Setup motor ID
    buffer_.ID = msg->data;

    // Data Marshalling
    DrivingMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, 0);
    ROS_INFO("MOTOR %d alarms cleared", buffer_.ID);
}

void DrivingMotors::commandsCB(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
    // Set motors rpm
    for (int i = 1; i < 5; i++)
    {
        DrivingMotors::setSpeed(this->motorID_[i], msg->data[i-1]);
    }
}


void DrivingMotors::Parser()
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

void DrivingMotors::clearBuffer()
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