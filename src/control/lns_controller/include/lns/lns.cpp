#include "lns.hpp"

LnsRobot::LnsRobot(ros::NodeHandle& nodehandle) : spinner_(6)
{
    // store the node handle passed to the hardware interface
    // in the private member variable
    n_ = nodehandle;
    spinner_.start();

    // Initialize the other private member variables
    pos[0] = 0.0; pos[1] = 0.0; pos[2] = 0.0; pos[3] = 0.0; pos[4] = 0.0; pos[5] = 0.0; pos[6] = 0.0; pos[7] = 0.0;
    vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0; vel[3] = 0.0; vel[4] = 0.0; vel[5] = 0.0; vel[6] = 0.0; vel[7] = 0.0;
    eff[0] = 0.0; eff[1] = 0.0; eff[2] = 0.0; eff[3] = 0.0; eff[4] = 0.0; eff[5] = 0.0; eff[6] = 0.0; eff[7] = 0.0;
    cmd[0] = 0.0; cmd[1] = 0.0; cmd[2] = 0.0; cmd[3] = 0.0; cmd[4] = 0.0; cmd[5] = 0.0; cmd[6] = 0.0; cmd[7] = 0.0;
    
    // Create joint state handles for driving motors
    hardware_interface::JointStateHandle state_driving_motor_fl("wheel_fl_joint", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_driving_motor_fr("wheel_fr_joint", &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_driving_motor_br("wheel_br_joint", &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_driving_motor_bl("wheel_bl_joint", &pos[3], &vel[3], &eff[3]);

    // Create joint state handles for steering motors
    hardware_interface::JointStateHandle state_steering_motor_fl("steering_fl_joint", &pos[4], &vel[4], &eff[4]);
    hardware_interface::JointStateHandle state_steering_motor_fr("steering_fr_joint", &pos[5], &vel[5], &eff[5]);
    hardware_interface::JointStateHandle state_steering_motor_br("steering_br_joint", &pos[6], &vel[6], &eff[6]);
    hardware_interface::JointStateHandle state_steering_motor_bl("steering_bl_joint", &pos[7], &vel[7], &eff[7]);

    // Register them
    jnt_state_.registerHandle(state_driving_motor_fl);
    jnt_state_.registerHandle(state_driving_motor_fr);
    jnt_state_.registerHandle(state_driving_motor_br);
    jnt_state_.registerHandle(state_driving_motor_bl);

    jnt_state_.registerHandle(state_steering_motor_fl);
    jnt_state_.registerHandle(state_steering_motor_fr);
    jnt_state_.registerHandle(state_steering_motor_br);
    jnt_state_.registerHandle(state_steering_motor_bl);
    registerInterface(&jnt_state_);

    // Create joint command handles and register them
    hardware_interface::JointHandle cmd_driving_motor_fl (state_driving_motor_fl, &cmd[0]);
    hardware_interface::JointHandle cmd_driving_motor_fr (state_driving_motor_fr, &cmd[1]);
    hardware_interface::JointHandle cmd_driving_motor_br (state_driving_motor_br, &cmd[2]);
    hardware_interface::JointHandle cmd_driving_motor_bl (state_driving_motor_bl, &cmd[3]);

    hardware_interface::JointHandle cmd_steering_motor_fl (state_steering_motor_fl, &cmd[4]);
    hardware_interface::JointHandle cmd_steering_motor_fr (state_steering_motor_fr, &cmd[5]);
    hardware_interface::JointHandle cmd_steering_motor_br (state_steering_motor_br, &cmd[6]);
    hardware_interface::JointHandle cmd_steering_motor_bl (state_steering_motor_bl, &cmd[7]);

    jnt_cmd_vel_.registerHandle(cmd_driving_motor_fl);
    jnt_cmd_vel_.registerHandle(cmd_driving_motor_fr);
    jnt_cmd_vel_.registerHandle(cmd_driving_motor_br);
    jnt_cmd_vel_.registerHandle(cmd_driving_motor_bl);
    registerInterface(&jnt_cmd_vel_);

    jnt_cmd_pos_.registerHandle(cmd_steering_motor_fl);
    jnt_cmd_pos_.registerHandle(cmd_steering_motor_fr);
    jnt_cmd_pos_.registerHandle(cmd_steering_motor_br);
    jnt_cmd_pos_.registerHandle(cmd_steering_motor_bl);
    registerInterface(&jnt_cmd_pos_);
    

    // Create a subscriber to get angle in rad
    steering_motors_sub_ = n_.subscribe("/steering_motors/feedback/rad", 1, &LnsRobot::steeringCB, this);
    // Create a subscriber to get rad/sec
    motor1_state_sub_ = n_.subscribe("/driving_pid/pid/motor1/state", 1, &LnsRobot::m1stateCB, this);
    motor2_state_sub_ = n_.subscribe("/driving_pid/pid/motor2/state", 1, &LnsRobot::m2stateCB, this);
    motor3_state_sub_ = n_.subscribe("/driving_pid/pid/motor3/state", 1, &LnsRobot::m3stateCB, this);
    motor4_state_sub_ = n_.subscribe("/driving_pid/pid/motor4/state", 1, &LnsRobot::m4stateCB, this);

    // Create a publisher to send data to driving motors
    driving_motors_pub_ = n_.advertise<std_msgs::Int64MultiArray>("/driving_motors/commands", 1);

    // Create a publisher to send data to steering motors
    steering_motor5_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor5/setpoint", 1);
    steering_motor6_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor6/setpoint", 1);
    steering_motor7_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor7/setpoint", 1);
    steering_motor8_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor8/setpoint", 1);

    // Driving PID
    driving_motor1_pub_ = n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor1/setpoint", 1);
    driving_motor2_pub_ = n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor2/setpoint", 1);
    driving_motor3_pub_ = n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor3/setpoint", 1);
    driving_motor4_pub_ = n_.advertise<std_msgs::Float64>("/driving_pid/pid/motor4/setpoint", 1);
}

LnsRobot::~LnsRobot()
{
    spinner_.stop();
}

void LnsRobot::read()
{
    /*
        
    */
    // TODO: Fix this in drivers
    pos[4] = -steering_rad_[0];
    pos[5] = -steering_rad_[1];
    pos[6] = -steering_rad_[2];
    pos[7] = -steering_rad_[3];
    
    vel[0] = driving_radsec_[0];
    vel[1] = driving_radsec_[1];
    vel[2] = driving_radsec_[2];
    vel[3] = driving_radsec_[3];
}

void LnsRobot::write(ros::Duration elapsed_time)
{
    motor1_rpm_.data = cmd[0];
    motor2_rpm_.data = cmd[1];
    motor3_rpm_.data = cmd[2];
    motor4_rpm_.data = cmd[3];
    driving_motor1_pub_.publish(motor1_rpm_);
    driving_motor2_pub_.publish(motor2_rpm_);
    driving_motor3_pub_.publish(motor3_rpm_);
    driving_motor4_pub_.publish(motor4_rpm_); 

    // TODO: Fix this in drivers
    motor5_rad_.data = -cmd[4];
    motor6_rad_.data = -cmd[5];
    motor7_rad_.data = -cmd[6];
    motor8_rad_.data = -cmd[7];
    steering_motor5_pub_.publish(motor5_rad_);
    steering_motor6_pub_.publish(motor6_rad_);
    steering_motor7_pub_.publish(motor7_rad_);
    steering_motor8_pub_.publish(motor8_rad_);    
}

void LnsRobot::m1stateCB(const std_msgs::Float64::ConstPtr& msg)
{
    driving_radsec_[0] = msg->data; 
}

void LnsRobot::m2stateCB(const std_msgs::Float64::ConstPtr& msg)
{
    driving_radsec_[1] = msg->data; 
}

void LnsRobot::m3stateCB(const std_msgs::Float64::ConstPtr& msg)
{
    driving_radsec_[2] = msg->data; 
}

void LnsRobot::m4stateCB(const std_msgs::Float64::ConstPtr& msg)
{
    driving_radsec_[3] = msg->data; 
}


void LnsRobot::steeringCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    steering_rad_[0] = msg->data[0];
    steering_rad_[1] = msg->data[1];
    steering_rad_[2] = msg->data[2];
    steering_rad_[3] = msg->data[3];
}
