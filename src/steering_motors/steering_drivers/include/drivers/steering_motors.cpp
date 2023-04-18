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

SteeringMotors::SteeringMotors(ros::NodeHandle n, ros::NodeHandle n1, ros::NodeHandle n2, ros::NodeHandle n3, ros::NodeHandle n4)
{
    /*
    Class Inicialization
    Assign the subscribers and publishers
    Assign Nodehandlers for the multithread queue & callbacks
    */

    this->n_ = n;
    this->n1_ = n1;
    this->n2_ = n2;
    this->n3_ = n3;
    this->n4_ = n4;

    this->alarm_monitor_ = this->n_.advertise<std_msgs::Int8MultiArray>("/steering_motors/alarm_monitor/status", 1);
    this->rpm_feedback_ = this->n_.advertise<std_msgs::Int64MultiArray>("/steering_motors/feedback/rpm", 1);
    this->alarm_clear_ = this->n_.subscribe("/steering_motors/alarm_monitor/clear_alarm", 1, &SteeringMotors::clearAlarmCB, this); 
    this->rad_feedback_ = this->n_.subscribe("/steering_motors/feedback/rad", 1, &SteeringMotors::radFeedbackCB, this); 
    
    this->n1_.setCallbackQueue(&callback_queue_wheel_1_);
    this->pidWheel_1_ = this->n1_.subscribe("/steering_motors/pid/motor5/control_effort", 1, &SteeringMotors::motor5CB, this);
    
    this->n2_.setCallbackQueue(&callback_queue_wheel_2_);
    this->pidWheel_2_ = this->n2_.subscribe("/steering_motors/pid/motor6/control_effort", 1, &SteeringMotors::motor6CB, this);

    this->n3_.setCallbackQueue(&callback_queue_wheel_3_);
    this->pidWheel_3_ = this->n3_.subscribe("/steering_motors/pid/motor7/control_effort", 1, &SteeringMotors::motor7CB, this);

    this->n4_.setCallbackQueue(&callback_queue_wheel_4_);
    this->pidWheel_4_ = this->n4_.subscribe("/steering_motors/pid/motor8/control_effort", 1, &SteeringMotors::motor8CB, this) ;

    
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

SteeringMotors::~SteeringMotors()
{

}


int SteeringMotors::connSocket()
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


void SteeringMotors::setPos(uint8_t motorID, double pos)
{
    /*
    Transform rpm to byte and send the speed command
    If the DLC is change from 0x08 feedback becomes unstable
    */

    // Clear the buffer
    SteeringMotors::clearBuffer();

    // Setup Command PID 219
    buffer_.PID = 0xDB;

    // Setup motor ID
    buffer_.ID = motorID;

    // Setup position bytes
    // Transform rad to pos 
    //pos_ = radTopos(motorID, rad); // no need to this, since this is the pid control effort
    pos_ = int(pos);
    // Hard limits
    int hard_top = 0;
    int hard_low = 0;
    hard_top = max_limit_pos_[motorID-5] + motor_offsets_[motorID-5];
    hard_low = min_limit_pos_[motorID-5] + motor_offsets_[motorID-5];

    if (pos_ > hard_top)
    {
        pos_ = hard_top;
    }
    if (pos_ < hard_low)
    {
        pos_ = hard_low;
    }

    // If pos are positive
    if (pos_ >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        buffer_.D4 = (pos_ >> 24) & 0xff;
        buffer_.D3 = (pos_ >> 16) & 0xff;
        buffer_.D2 = (pos_ >> 8) & 0xff; 
        buffer_.D1 = pos_ & 0xff; 
    }

    // If pos are negative
    else
    {
        int neg_dec;
        neg_dec = 4294967295 - abs(pos_); // 6 x FF- pos
        // 16 bytes = 8bytes0 8bytes1 
        buffer_.D4 = (neg_dec >> 24) & 0xff;
        buffer_.D3 = (neg_dec >> 16) & 0xff;
        buffer_.D2 = (neg_dec >> 8) & 0xff; 
        buffer_.D1 = neg_dec & 0xff;
    }

    buffer_.D6 = 0;
    buffer_.D5 = 0;

    // Data Marshalling
    SteeringMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, MSG_DONTWAIT);

}

int SteeringMotors::alarmMonitor()
{
    /*
    Check the controller status.
    Reply: DATA(BIT0~7)
    BIT0 : ALARM, (1-> alarm status, 0->normal)
    BIT1 : CTRL_FAIL, Speed control fail
    BIT2 : OVER_VOLT, Over voltage
    BIT3 : OVER_TEMP, Over temperature
    BIT4 : OVER_LOAD, Overload
    BIT5 : HALL_FAIL, Hall sensor or encoder fail
    BIT6 : INV_VEL, Motor speed inversed
    BIT7 : STALL, motor not moved

    If the DLC is change from 0x08 feedback becomes unstable
    */

    // Clear the buffer
    SteeringMotors::clearBuffer();

    // Setup Command PID 34
    buffer_.PID = 0x04;
    buffer_.D1 = 0x2B; //22??

    // Create vector to store data
    std::vector<int> vec_alarms (4);

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        SteeringMotors::Parser();

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

int SteeringMotors::feedback()
{
    /*
    Calculates the RPM of the motor.
    Then publish the rpm topic

    If the DLC is change from 0x08 feedback becomes unstable
    */

    // Clear the buffer
    SteeringMotors::clearBuffer();

    // Setup Command PID 138
    buffer_.PID = 0x04;
    buffer_.D1 = 0x8A;

    // Create vector to store data
    std::vector<int> vec_rpm (4);

    for (int i = 1; i < 5; i++)
    {
        // Setup motor ID
        buffer_.ID = this->motorID_[i];

        // Data Marshalling
        SteeringMotors::Parser();

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
            // bytes_in_[4] is motor ID 5,6,7,8 so we susbtract -5 to start the array from 0
            vec_rpm[bytes_in_[4] - 5] = SteeringMotors::byteTorpm(bytes_in_[6], bytes_in_[7]);
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

    // Publish topics
    rpm_feedback_.publish(rpms_);

    //clear stuff
    vec_rpm.clear();
    rpms_.data.clear();
    
    return 0;
}

int SteeringMotors::byteTorpm(uint8_t byte0, uint8_t byte1)
{
    /*
    Transform the bytes received into the motor rpm
    */

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

int SteeringMotors::radTopos(uint8_t motorID, double rad)
{
    double m;
    int pos;
    m = (max_limit_pos_[motorID-5] - min_limit_pos_[motorID-5])/M_PI;
    pos = int(m*rad);
    return pos;
}

void SteeringMotors::calibrationRoutine()
{
    /*
    The zero position will be given by the starting position 
    of the wheel when the motors are powered, those this routine
    find the offset for each motor.
    */

    ROS_INFO("Starting Calibration");

    // Alignement flag
    bool align = false;
    // Deadband
    double deadband_rad = 0.00174; // 0.1 degree
    // Offset
    int j = 0;

    for (int i = 1; i < 5; i++)
    {
        while (!align)
        {
            if (fabs(motor_angles_[i-1]) <= deadband_rad)
            {
                align = true;
                break;
            }

            if(motor_angles_[i-1] > 0)
            {
                j--;
                SteeringMotors::setPos(motorID_[i], j);
            }
            else if (motor_angles_[i-1] < 0)
            {
                j++;
                SteeringMotors::setPos(motorID_[i], j);
            }
        }
        // Save offsets
        motor_offsets_[i-1] = j;

        // Reset values
        align = false;
        j = 0;
    }

    ROS_INFO("Calibration Successed");
}

void SteeringMotors::clearAlarmCB(const std_msgs::Int8::ConstPtr& msg)
{
    /* 
    Clear the alarm of the motor controller in msg
    Just 1 motor controller at time
    */

    // Clear the buffer
    SteeringMotors::clearBuffer();

    // Setup Command PID 12
    buffer_.PID = 0x0C;

    // Setup motor ID
    buffer_.ID = msg->data;

    // Data Marshalling
    SteeringMotors::Parser();

    // Send command
    send(client_, bytes_out_, 13, MSG_DONTWAIT);
    ROS_INFO("MOTOR %d alarms cleared", buffer_.ID);
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
    SteeringMotors::setPos(this->motorID_[1], msg->data);
}


void SteeringMotors::motor6CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 6
    SteeringMotors::setPos(this->motorID_[2], msg->data);
}

void SteeringMotors::motor7CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 7
    SteeringMotors::setPos(this->motorID_[3], msg->data);
}

void SteeringMotors::motor8CB(const std_msgs::Float64::ConstPtr& msg)
{
    //Motor 8
    SteeringMotors::setPos(this->motorID_[4], msg->data);
}


void SteeringMotors::emergencyStop()
{
    /*
    Set motors rpm to 0
    */

    for (int i = 1; i < 5; i++)
    {
        SteeringMotors::setPos(this->motorID_[i], 0);
    }

    // Close the socket
    close(client_);
}


void SteeringMotors::spinners()
{
    /*
    Create the spinner queue for each callback
    */

    std::thread spinner_thread_wheel1([&]()
    {
        this->spinner_1_.spin(&callback_queue_wheel_1_);
    });

    std::thread spinner_thread_wheel2([&]()
    {
        this->spinner_2_.spin(&callback_queue_wheel_2_);
    });

    std::thread spinner_thread_wheel3([&](){
        this->spinner_3_.spin(&callback_queue_wheel_3_);
    });

    std::thread spinner_thread_wheel4([&](){
        this->spinner_4_.spin(&callback_queue_wheel_4_);
    });

    ros::spin(); // spin the n

    // Spin
    spinner_thread_wheel1.join();
    spinner_thread_wheel2.join();
    spinner_thread_wheel3.join();
    spinner_thread_wheel4.join();
}

void SteeringMotors::Parser()
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

void SteeringMotors::clearBuffer()
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