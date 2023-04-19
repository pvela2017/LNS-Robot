#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_


#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

class LnsRobot : public hardware_interface::RobotHW
{
private:
    ros::NodeHandle n_;

    ros::Publisher driving_motors_pub_;
    ros::Subscriber driving_motors_sub_;
    ros::Publisher steering_motor5_pub_;
    ros::Publisher steering_motor6_pub_;
    ros::Publisher steering_motor7_pub_;
    ros::Publisher steering_motor8_pub_;
    ros::Subscriber steering_motors_sub_;

    std_msgs::Int64MultiArray driving_command_rpms_;
    std_msgs::Float64 motor5_rad_;
    std_msgs::Float64 motor6_rad_;
    std_msgs::Float64 motor7_rad_;
    std_msgs::Float64 motor8_rad_;
    //std_msgs::Int64MultiArray steering_command_rpms_; TODO
    
    hardware_interface::JointStateInterface jnt_state_;
    hardware_interface::VelocityJointInterface jnt_cmd_vel_;
    hardware_interface::PositionJointInterface jnt_cmd_pos_;
    
    joint_limits_interface::JointLimits jnt_limits_;
    joint_limits_interface::VelocityJointSaturationInterface jnt_vel_sat_;

    double pos[8];
    double vel[8];
    double eff[8];
    double cmd[8];

    // From callback
    double driving_radsec_[4];
    double steering_rad_[4];
    
    int radsecTorpm(double);
    double rpmToradsec(int);
    double angleWrapper(double);
    void drivingCB(const std_msgs::Float64MultiArray::ConstPtr&);
    void steeringCB(const std_msgs::Float64MultiArray::ConstPtr&);

public:
    LnsRobot(ros::NodeHandle& nodehandle);
    ~LnsRobot();
    void read();
    void write(ros::Duration elapsed_time);
    
};

#endif // ROBOT_HW_INTERFACE_H_
