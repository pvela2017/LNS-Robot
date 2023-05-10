#include "lns.hpp"

LnsRobot::LnsRobot(ros::NodeHandle& nodehandle)
{
    // store the node handle passed to the hardware interface
    // in the private member variable
    n_ = nodehandle;

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
    
    /*
    // Create joint limits handles and register them
    joint_limits_interface::getJointLimits("driving_motor_fl", n_, jnt_limits_);
    joint_limits_interface::VelocityJointSaturationHandle limit_driving_motor_fl(cmd_driving_motor_fl, jnt_limits_);

    joint_limits_interface::getJointLimits("driving_motor_fr", n_, jnt_limits_);
    joint_limits_interface::VelocityJointSaturationHandle limit_driving_motor_fr(driving_motor_fr, jnt_limits_);

    joint_limits_interface::getJointLimits("driving_motor_br", n_, jnt_limits_);
    joint_limits_interface::VelocityJointSaturationHandle limit_driving_motor_br(driving_motor_br, jnt_limits_);

    joint_limits_interface::getJointLimits("driving_motor_bl", n_, jnt_limits_);
    joint_limits_interface::VelocityJointSaturationHandle limit_driving_motor_br(driving_motor_bl, jnt_limits_);



    jnt_vel_sat_.registerHandle(limit_driving_motor_fl);
    jnt_vel_sat_.registerHandle(limit_steering_motor_fr);
    jnt_vel_sat_.registerHandle(limit_driving_motor_br);
    jnt_vel_sat_.registerHandle(limit_driving_motor_br);
    registerInterface(&jnt_vel_sat_);*/
    
    // Create a subscriber to get rad/sec
    driving_motors_sub_ = n_.subscribe("/driving_motors/feedback/angular/radsec", 1, &LnsRobot::drivingCB, this);
    // Create a subscriber to get angle in rad
    steering_motors_sub_ = n_.subscribe("/steering_motors/feedback/rad", 1, &LnsRobot::steeringCB, this);

    // Create a publisher to send data to driving motors
    driving_motors_pub_ = n_.advertise<std_msgs::Int64MultiArray>("/driving_motors/commands", 1);
    // Create a publisher to send data to steering motors
    steering_motor5_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor5/setpoint", 1);
    steering_motor6_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor6/setpoint", 1);
    steering_motor7_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor7/setpoint", 1);
    steering_motor8_pub_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor8/setpoint", 1);
}

LnsRobot::~LnsRobot()
{

}

void LnsRobot::read()
{
    // TODO: Fix this in drivers
    pos[4] = -steering_rad_[0];
    pos[5] = -steering_rad_[1];
    pos[6] = -steering_rad_[2];
    pos[7] = -steering_rad_[3];
    
    // TODO: Fix this in drivers
    vel[0] = driving_radsec_[0];
    vel[1] = -driving_radsec_[1];
    vel[2] = -driving_radsec_[2];
    vel[3] = driving_radsec_[3];
}

void LnsRobot::write(ros::Duration elapsed_time)
{
    //jnt_vel_sat.enforceLimits(elapsed_time);

    // Driving motors
    // Create vector to store driving data
    std::vector<int> vec_rpm (4);

    vec_rpm[0] = radsecTorpm(cmd[0]);
    vec_rpm[1] = radsecTorpm(cmd[1]);
    vec_rpm[2] = radsecTorpm(cmd[2]);
    vec_rpm[3] = radsecTorpm(cmd[3]);

    // Push data into data blob
    std::vector<int>::const_iterator itr, end(vec_rpm.end());
    for(itr = vec_rpm.begin(); itr!= end; ++itr) 
    {
        driving_command_rpms_.data.push_back(*itr); 
    }
    
    // Publish topic
    driving_motors_pub_.publish(driving_command_rpms_);

    // Clear stuff
    vec_rpm.clear();
    driving_command_rpms_.data.clear();


    // Steering motors
    /*
    
    TODO: 
        LOW LEVEL CONTROLLER IN HARDWARE

    // Create vector to store steering data
    std::vector<int> vec_rpm2 (4);

    vec_rpm2[0] = radsecTorpm(cmd[4]);
    vec_rpm2[1] = radsecTorpm(cmd[5]);
    vec_rpm2[2] = radsecTorpm(cmd[6]);
    vec_rpm2[3] = radsecTorpm(cmd[7]);

    // Push data into data blob
    std::vector<int>::const_iterator itr2, end2(vec_rpm2.end());
    for(itr2 = vec_rpm2.begin(); itr!= end2; ++itr2) 
    {
        steering_command_rpms_.data.push_back(*itr2); 
    }

    // Publish topic
    steering_motors_pub_.publish(steering_command_rpms_);

    // Clear stuff
    vec_rpm2.clear();
    steering_command_rpms_.data.clear();
    */
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

void LnsRobot::drivingCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    driving_radsec_[0] = msg->data[0];
    driving_radsec_[1] = msg->data[1];
    driving_radsec_[2] = msg->data[2];
    driving_radsec_[3] = msg->data[3];    
}


void LnsRobot::steeringCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    steering_rad_[0] = msg->data[0];
    steering_rad_[1] = msg->data[1];
    steering_rad_[2] = msg->data[2];
    steering_rad_[3] = msg->data[3];
}

int LnsRobot::radsecTorpm(double radsec)
{
    double wheel_rpm = 0;
    int motor_rpm = 0;

    wheel_rpm = (radsec *60.0)/(2*M_PI);
    motor_rpm = int(wheel_rpm*30); // DRIVING_GEAR_BOX_RATIO from robot dic
    return motor_rpm;
}

double LnsRobot::rpmToradsec(int rpm)
{
    double radsec;
    radsec = rpm*9.5492;
    return radsec;
}


double LnsRobot::angleWrapper(double rad)
{
    if (rad > 3.141592)
    {
        rad -= 2.*3.141592;
    }       
    return rad;
}
