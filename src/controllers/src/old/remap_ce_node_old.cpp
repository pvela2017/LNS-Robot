/*
Transform the message from the control effort topic
to bytes.

node: remap_node
Subscribe to: /steeringmotors/commands
Publish to :  /wheel_1_steering_pid/setpoint
              /wheel_2_steering_pid/setpoint
              /wheel_3_steering_pid/setpoint
              /wheel_4_steering_pid/setpoint


by Pablo
Last review: 2022/07/29
*/


#include "ros/ros.h"
#include <thread>
#include <ros/callback_queue.h>

#include "std_msgs/Float64.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"

// Motor IDs
#define motorId1 0x01
#define motorId2 0x02
#define motorId3 0x03
#define motorId4 0x04


// Function declaration
int* rpmTobyte(double);
void callback_pidWheel_1(const std_msgs::Float64&);
void callback_pidWheel_2(const std_msgs::Float64&);
void callback_pidWheel_3(const std_msgs::Float64&);
void callback_pidWheel_4(const std_msgs::Float64&);

// Globals

ros::NodeHandle np;
ros::Publisher pub = np.advertise<std_msgs::ByteMultiArray>("/socket2/send", 1);

// Function definition 
void callback_pidWheel_1(const std_msgs::Float64& value)
{
    //ros::NodeHandle np;
    //ros::Publisher pub = np.advertise<std_msgs::ByteMultiArray>("/socket2/send", 1);
    int *temp_p;

    std_msgs::ByteMultiArray msg;
    msg.data[0] = motorId1;


    temp_p = rpmTobyte(value.data);
    msg.data[1] = temp_p[0];
    msg.data[2] = temp_p[3];

    pub.publish(msg);
}


void callback_pidWheel_2(const std_msgs::Float64& value)
{
    ros::NodeHandle np;
    ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //pub.publish(msg);

}

void callback_pidWheel_3(const std_msgs::Float64& value)
{
    ros::NodeHandle np;
    ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //pub.publish(msg);

}

void callback_pidWheel_4(const std_msgs::Float64& value)
{
    ros::NodeHandle np;
    ros::Publisher pub = np.advertise<std_msgs::String>("/socket2/send", 1);

    //pub.publish(msg);
}


int* rpmTobyte(double recv_rpm)
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




int main(int argc, char **argv)
{
    ros::init(argc, argv, "steeringmotors_message");
    // n1 goes to the regular roscpp queue handler
    ros::NodeHandle n1;
    ros::Subscriber pidWheel_1 = n1.subscribe("/wheel_1_steering_pid/control_effort", 1, callback_pidWheel_1);


    ros::NodeHandle n2;
    ros::CallbackQueue callback_queue_wheel_2;
    n2.setCallbackQueue(&callback_queue_wheel_2);
    ros::Subscriber pidWheel_2 = n2.subscribe("/wheel_2_steering_pid/control_effort", 1, callback_pidWheel_2);


    ros::NodeHandle n3;
    ros::CallbackQueue callback_queue_wheel_3;
    n2.setCallbackQueue(&callback_queue_wheel_3);
    ros::Subscriber pidWheel_3 = n3.subscribe("/wheel_3_steering_pid/control_effort", 1, callback_pidWheel_3);


    ros::NodeHandle n4;
    ros::CallbackQueue callback_queue_wheel_4;
    n4.setCallbackQueue(&callback_queue_wheel_4);
    ros::Subscriber pidWheel_4 = n4.subscribe("/wheel_4_steering_pid/control_effort", 1, callback_pidWheel_4);


    std::thread spinner_thread_wheel2([&callback_queue_wheel_2](){
        ros::SingleThreadedSpinner spinner_2;
        spinner_2.spin(&callback_queue_wheel_2);
    });

    std::thread spinner_thread_wheel3([&callback_queue_wheel_3](){
        ros::SingleThreadedSpinner spinner_3;
        spinner_3.spin(&callback_queue_wheel_3);
    });

    std::thread spinner_thread_wheel4([&callback_queue_wheel_4](){
        ros::SingleThreadedSpinner spinner_4;
        spinner_4.spin(&callback_queue_wheel_4);
    });


    ros::spin(); // spin the n1
    // Spin
    spinner_thread_wheel2.join();
    spinner_thread_wheel3.join();
    spinner_thread_wheel4.join();

    return 0;


}





