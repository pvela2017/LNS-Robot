#include <ros/ros.h>
#include <driving_motors.hpp>


/*
int rate_alarm = 1;
int rate_feedback = 20;



void alarmCB(const ros::TimerEvent& event)
{
    motors.alarmMonitor();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::NodeHandle n;

    DrivingMotors motors(n);
    motors.connSocket();


    while (ros::ok)
    {

        motors.alarmMonitor();
        motors.feedback();

        ros::spinOnce();
        loop_rate.sleep();
    }

}*/




int main(int argc, char **argv)
{
    ros::init(argc, argv, "driving_motors");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    DrivingMotors motors(n);
    motors.connSocket();


    while (ros::ok)
    {

        //motors.alarmMonitor();
        motors.feedback();

        ros::spinOnce();
        loop_rate.sleep();
    }

}

