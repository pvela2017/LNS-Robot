
#include "kinematics.hpp"

Kinematics::Kinematics(ros::NodeHandle n)     
{
  this->n_ = n;
  cmd_vel_sub_ = n_.subscribe("cmd_vel", 1, &Kinematics::cmdVelCB, this);
  /// Publisher driving and steering commands interface
  driving_rpms_ = n_.advertise<std_msgs::Int64MultiArray>("/driving_motors/commands", 1);
  steering_motor5_angle_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor5/setpoint", 1);
  steering_motor6_angle_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor6/setpoint", 1);
  steering_motor7_angle_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor7/setpoint", 1);
  steering_motor8_angle_ = n_.advertise<std_msgs::Float64>("/steering_motors/pid/motor8/setpoint", 1);

  // Robot parameters
  track_ = 1.02;
  wheel_steering_y_offset_ = 0;
  wheel_radius_ = 0.4;
  wheel_base_ = 0.99;
}

Kinematics::~Kinematics()
{

}

void Kinematics::cmdVelCB(const geometry_msgs::Twist& command)
{
  // Check if the command is valid
  if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
  {
    ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
    return;
  }
  twist_command_.angular.z   = command.angular.z;
  twist_command_.linear.x   = command.linear.x;
  twist_command_.linear.y   = command.linear.y;

  Kinematics::updateCommand();
}


void Kinematics::updateCommand()
{
  //const double cmd_dt(period.toSec());
  const double steering_track = track_-2*wheel_steering_y_offset_;
  double vel_left_front = 0, vel_right_front = 0;
  double vel_left_rear = 0, vel_right_rear = 0;
  double front_left_steering = 0, front_right_steering = 0;
  double rear_left_steering = 0, rear_right_steering = 0;

  // TODO
  // Limit velocities and accelerations:
  //limiter_lin_.limit(twist_command_.linear.x, last0_cmd_.linear.x, last1_cmd_.linear.x, cmd_dt);
  //limiter_ang_.limit(twist_command_.angular.z, last0_cmd_.angular.z, last1_cmd_.angular.z, cmd_dt);

  //last1_cmd_ = last0_cmd_;
  //last0_cmd_ = twist_command_;


  // Compute wheels velocities:
  if ((fabs(twist_command_.linear.x) > 0.001) || (fabs(twist_command_.linear.y) > 0.001) ||
              (fabs(twist_command_.angular.z) > 0.001))
          {
              double a = twist_command_.linear.y - twist_command_.angular.z * wheel_base_ / 2;
              double b = twist_command_.linear.y + twist_command_.angular.z * wheel_base_ / 2;
              double c = twist_command_.linear.x - twist_command_.angular.z * steering_track / 2;
              double d = twist_command_.linear.x + twist_command_.angular.z * steering_track / 2;

              vel_left_front = sqrt(pow(b, 2) + pow(c, 2)) / wheel_radius_;
              vel_right_front = sqrt(pow(b, 2) + pow(d, 2)) / wheel_radius_;
              vel_left_rear = sqrt(pow(a, 2) + pow(c, 2)) / wheel_radius_;
              vel_right_rear = sqrt(pow(a, 2) + pow(d, 2)) / wheel_radius_;

              front_left_steering = atan2(b, c);
              front_right_steering = atan2(b, d);
              rear_left_steering = atan2(a, c);
              rear_right_steering = atan2(a, d);
          }


  /*
  if(fabs(twist_command_.linear.x) > 0.001)
  {
    const double vel_steering_offset = (twist_command_.angular.z*wheel_steering_y_offset_)/wheel_radius_;
    const double sign = copysign(1.0, twist_command_.linear.x);

    vel_left_front  = sign * std::hypot((twist_command_.linear.x - twist_command_.angular.z*steering_track/2),
                                        (wheel_base_*twist_command_.angular.z/2.0)) / wheel_radius_
                      - vel_steering_offset;
    vel_right_front = sign * std::hypot((twist_command_.linear.x + twist_command_.angular.z*steering_track/2),
                                        (wheel_base_*twist_command_.angular.z/2.0)) / wheel_radius_
                      + vel_steering_offset;
    vel_left_rear = sign * std::hypot((twist_command_.linear.x - twist_command_.angular.z*steering_track/2),
                                      (wheel_base_*twist_command_.angular.z/2.0)) / wheel_radius_
                    - vel_steering_offset;
    vel_right_rear = sign * std::hypot((twist_command_.linear.x + twist_command_.angular.z*steering_track/2),
                                       (wheel_base_*twist_command_.angular.z/2.0)) / wheel_radius_
                     + vel_steering_offset;
  }

  // Compute steering angles
  if(fabs(2.0*twist_command_.linear.x) > fabs(twist_command_.angular.z*steering_track) && fabs(twist_command_.linear.y) < 0.001)
  {
    front_left_steering = atan(twist_command_.angular.z*wheel_base_ /
                                (2.0*twist_command_.linear.x - twist_command_.angular.z*steering_track));
    front_right_steering = atan(twist_command_.angular.z*wheel_base_ /
                                 (2.0*twist_command_.linear.x + twist_command_.angular.z*steering_track));
     rear_left_steering = -front_left_steering;
     rear_right_steering = -front_right_steering;
  }
  else if(fabs(twist_command_.linear.x) > 0.001 && fabs(twist_command_.linear.y) < 0.001)
  {
    front_left_steering = copysign(M_PI_2, twist_command_.angular.z);
    front_right_steering = copysign(M_PI_2, twist_command_.angular.z);
    rear_left_steering = -front_left_steering;
    rear_right_steering = -front_right_steering;
  }
  else if(fabs(twist_command_.linear.x) > 0.001 && fabs(twist_command_.linear.y) >= 0.001 && fabs(twist_command_.angular.z) < 0.001)
  {
    front_left_steering = atan(twist_command_.linear.y / twist_command_.linear.x);
    front_right_steering = atan(twist_command_.linear.y / twist_command_.linear.x);
    rear_left_steering = front_left_steering;
    rear_right_steering = front_right_steering;        
  }*/

  // Transform speed to rpm
  vel_left_front = Kinematics::speedTorpm(vel_left_front);
  vel_right_front = Kinematics::speedTorpm(vel_right_front);
  vel_right_rear = Kinematics::speedTorpm(vel_right_rear);
  vel_left_rear = Kinematics::speedTorpm(vel_left_rear);

  // Limit rpm
  vel_left_front = Kinematics::rpmLimit(vel_left_front);
  vel_right_front = Kinematics::rpmLimit(vel_right_front);
  vel_right_rear = Kinematics::rpmLimit(vel_right_rear);
  vel_left_rear = Kinematics::rpmLimit(vel_left_rear);
    

  // Create vector to store data
  std::vector<int> vec_rpm (4);

  // Set values
  vec_rpm[0] = (int)vel_left_front;
  vec_rpm[1] = (int)vel_right_front;
  vec_rpm[2] = (int)vel_right_rear;
  vec_rpm[3] = (int)vel_left_rear;

  //push data into data blob
  std::vector<int>::const_iterator itr, end(vec_rpm.end());
  for(itr = vec_rpm.begin(); itr!= end; ++itr) 
  {
      rpms_.data.push_back(*itr); 
  }

  // Publish topics
  driving_rpms_.publish(rpms_);

  //clear stuff
  vec_rpm.clear();
  rpms_.data.clear();


  // Limit angles 
  front_left_steering = Kinematics::angleLimit(front_left_steering);
  front_right_steering = Kinematics::angleLimit(front_right_steering);
  rear_left_steering = Kinematics::angleLimit(rear_left_steering);
  rear_right_steering = Kinematics::angleLimit(rear_right_steering);

  // Set values
  motor5_angle_.data = front_left_steering;
  motor6_angle_.data = front_right_steering;
  motor7_angle_.data = rear_right_steering;
  motor8_angle_.data = rear_left_steering;

  // Publish topics
  steering_motor5_angle_.publish(motor5_angle_);
  steering_motor6_angle_.publish(motor6_angle_);
  steering_motor7_angle_.publish(motor7_angle_);
  steering_motor8_angle_.publish(motor8_angle_);

}


double Kinematics::angleLimit(double angle)
{
  if (angle > 2.1)
  {
    angle = 2.1;
  }
  if (angle < -2.1)
  {
    angle = -2.1;
  }
  return angle;
}


double Kinematics::speedTorpm(double speed)
{
  double wheel_rpm = 0;
  double motor_rpm = 0;
  // Transform from m/s linear velocity to motor rpm
  wheel_rpm = (speed*60)/(2*0.4*M_PI_2); // WHEELS_RADIUS from robot dic
  motor_rpm = wheel_rpm*30; // DRIVING_GEAR_BOX_RATIO from robot dic
  return motor_rpm;
}

double Kinematics::rpmLimit(double rpm)
{
  if (rpm > 10)
  {
    rpm = 10;
  }
  if (rpm < -10)
  {
    rpm = -10;
  }
  return rpm;
}