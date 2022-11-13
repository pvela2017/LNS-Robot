#include <sub_pub.hpp>

/*

Get the control effort of each pid controller and gather them 
to publish them in only one topic

node: steering_commands
Subscribe to: /wheel_1_steering_pid/control_effort
              /wheel_2_steering_pid/control_effort
              /wheel_3_steering_pid/control_effort
              /wheel_4_steering_pid/control_effort
Publish to :  /socket2/send


by Pablo
Last review: 2022/09/25

*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_commands");
    ros::NodeHandle n1, n2, n3, n4, np;
    sub_pub steering_motors(n1, n2, n3, n4, np);
    steering_motors.spinners();


    return 0;


}





