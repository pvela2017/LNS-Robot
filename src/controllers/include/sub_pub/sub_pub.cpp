#include "sub_pub.hpp"


sub_pub::sub_pub(ros::NodeHandle n1, ros::NodeHandle n2, ros::NodeHandle n3, ros::NodeHandle n4, ros::NodeHandle np)
{

    this->_n1 = n1;
    this->_n2 = n2;
    this->_n3 = n3;
    this->_n4 = n4;
    this->_np = np;

    

    this->_pidWheel_1 = this->_n1.subscribe("/wheel_1_steering_pid/control_effort", 1, &sub_pub::_callback_pidWheel_1, this);

    this->_n2.setCallbackQueue(&_callback_queue_wheel_2);
    this->_pidWheel_2 = this->_n2.subscribe("/wheel_2_steering_pid/control_effort", 1, &sub_pub::_callback_pidWheel_2, this);

    this->_n3.setCallbackQueue(&_callback_queue_wheel_3);
    this->_pidWheel_3 = this->_n3.subscribe("/wheel_3_steering_pid/control_effort", 1, &sub_pub::_callback_pidWheel_3, this);

    this->_n4.setCallbackQueue(&_callback_queue_wheel_4);
    this->_pidWheel_4 = this->_n4.subscribe("/wheel_4_steering_pid/control_effort", 1, &sub_pub::_callback_pidWheel_4, this) ;

    this->_pub = this->_np.advertise<std_msgs::Int64MultiArray>("/socket2/send", 1);

}

sub_pub::~sub_pub() 
{
}

int* sub_pub::_rpmTobyte(double recv_rpm)
{
    int rpm = (int)recv_rpm;
    static int bytes2return[2];
    // If rpm are positive
    if (rpm >= 0)
    {
        // 16 bytes = 8bytes0 8bytes1 
        bytes2return[1] = (rpm >> 8) & 0xff; //byte0
        bytes2return[0] = rpm & 0xff; //byte1
        return bytes2return;
    }

    // If rpm are negative
    else
    {
        int neg_dec;
        neg_dec = 65535 - abs(rpm); // FFFF- rpm
        // 16 bytes = 8bytes0 8bytes1 
        bytes2return[1] = (neg_dec >> 8) & 0xff;
        bytes2return[0] = neg_dec & 0xff;
        return bytes2return;
    }

}

void sub_pub::spinners(void)
{
    std::thread spinner_thread_wheel2([&]()
    {
        this->_spinner_2.spin(&_callback_queue_wheel_2);
    });

    std::thread spinner_thread_wheel3([&](){
        this->_spinner_3.spin(&_callback_queue_wheel_3);
    });

    std::thread spinner_thread_wheel4([&](){
        this->_spinner_4.spin(&_callback_queue_wheel_4);
    });

    ros::spin(); // spin the n1

    // Spin
    spinner_thread_wheel2.join();
    spinner_thread_wheel3.join();
    spinner_thread_wheel4.join();
}


void sub_pub::_callback_pidWheel_1(const std_msgs::Float64& value)
{
    //ros::NodeHandle np;
    //ros::Publisher pub = np.advertise<std_msgs::Int64MultiArray>("/socket2/send", 1);

    //                                     MotorID  PID130  b0   b1
    int setup_array[13] = {0x00, 0x00, 0x00, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Get rpm bytes
    int *temp_p;
    temp_p = _rpmTobyte(value.data);
    // Setup array
    setup_array[5] = temp_p[0];
    setup_array[6] = temp_p[1];

    // Create and publish message
    std_msgs::Int64MultiArray msg;
    for (int e : setup_array)
    {
        msg.data.push_back(e);
    }
    this->_pub.publish(msg);
}


void sub_pub::_callback_pidWheel_2(const std_msgs::Float64& value)
{
    //ros::NodeHandle np;
    //ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //                                     MotorID  PID130  b0   b1
    int setup_array[13] = {0x00, 0x00, 0x00, 0x02, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Get rpm bytes
    int *temp_p;
    temp_p = _rpmTobyte(value.data);
    // Setup array
    setup_array[5] = temp_p[0];
    setup_array[6] = temp_p[1];

    // Create and publish message
    std_msgs::Int64MultiArray msg;
    for (int e : setup_array)
    {
        msg.data.push_back(e);
    }
    this->_pub.publish(msg);

}

void sub_pub::_callback_pidWheel_3(const std_msgs::Float64& value)
{
    //ros::NodeHandle np;
    //ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //                                     MotorID  PID130  b0   b1
    int setup_array[13] = {0x00, 0x00, 0x00, 0x03, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Get rpm bytes
    int *temp_p;
    temp_p = _rpmTobyte(value.data);
    // Setup array
    setup_array[5] = temp_p[0];
    setup_array[6] = temp_p[1];

    // Create and publish message
    std_msgs::Int64MultiArray msg;
    for (int e : setup_array)
    {
        msg.data.push_back(e);
    }
    this->_pub.publish(msg);
}

void sub_pub::_callback_pidWheel_4(const std_msgs::Float64& value)
{
    //ros::NodeHandle np;
    //ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //                                     MotorID  PID130  b0   b1
    int setup_array[13] = {0x00, 0x00, 0x00, 0x03, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Get rpm bytes
    int *temp_p;
    temp_p = _rpmTobyte(value.data);
    // Setup array
    setup_array[5] = temp_p[0];
    setup_array[6] = temp_p[1];

    // Create and publish message
    std_msgs::Int64MultiArray msg;
    for (int e : setup_array)
    {
        msg.data.push_back(e);
    }
    this->_pub.publish(msg);
}








