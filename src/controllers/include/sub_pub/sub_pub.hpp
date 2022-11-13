/*

Get the control effort of each pid controller and gather them 
to publish them in only one topic

The message is an Int Multi Array.


node: steering_commands
Subscribe to: /wheel_1_steering_pid/control_effort
              /wheel_2_steering_pid/control_effort
              /wheel_3_steering_pid/control_effort
              /wheel_4_steering_pid/control_effort
Publish to :  /socket2/send


by Pablo
Last review: 2022/09/25
*/


#include "ros/ros.h"
#include <thread>
#include <string>
#include <ros/callback_queue.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/String.h"

// Motor IDs
#define motorId1 0x01
#define motorId2 0x02
#define motorId3 0x03
#define motorId4 0x04



class sub_pub
{
    public:
        sub_pub(ros::NodeHandle n1, ros::NodeHandle n2, ros::NodeHandle n3, ros::NodeHandle n4, ros::NodeHandle np);
        ~sub_pub();

        // Spinner
        void spinners(void);

    private:
        // Variables
        //string _pubTopic;
        // Nodehandlers
        ros::NodeHandle _n1;
        ros::NodeHandle _n2;
        ros::NodeHandle _n3;
        ros::NodeHandle _n4;
        ros::NodeHandle _np;
        // Callbacks queues
        ros::CallbackQueue _callback_queue_wheel_2;
        ros::CallbackQueue _callback_queue_wheel_3;
        ros::CallbackQueue _callback_queue_wheel_4;
        // Subscribers
        ros::Subscriber _pidWheel_1;
        ros::Subscriber _pidWheel_2;
        ros::Subscriber _pidWheel_3;
        ros::Subscriber _pidWheel_4;
        // Publisher
        ros::Publisher _pub;
        // Spinners
        ros::SingleThreadedSpinner _spinner_2;
        ros::SingleThreadedSpinner _spinner_3;
        ros::SingleThreadedSpinner _spinner_4;
        // Methods
        int* _rpmTobyte(double);
        // Callbacks
        void _callback_pidWheel_1(const std_msgs::Float64&);
        void _callback_pidWheel_2(const std_msgs::Float64&);
        void _callback_pidWheel_3(const std_msgs::Float64&);
        void _callback_pidWheel_4(const std_msgs::Float64&);
};










