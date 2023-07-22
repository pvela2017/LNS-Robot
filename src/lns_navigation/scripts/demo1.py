#!/usr/bin/env python3

"""

rotate in position

CTRL-C to quit

by Pablo
Last review: 2022/07/26


"""

import time
import rospy
from std_msgs.msg import Float64



# Create node
rospy.init_node('demo')
pub_driving_1 = rospy.Publisher('/driving_pid/pid/motor1/setpoint', Float64, queue_size = 5)
pub_driving_2 = rospy.Publisher('/driving_pid/pid/motor2/setpoint', Float64, queue_size = 5)
pub_driving_3 = rospy.Publisher('/driving_pid/pid/motor3/setpoint', Float64, queue_size = 5)
pub_driving_4 = rospy.Publisher('/driving_pid/pid/motor4/setpoint', Float64, queue_size = 5)

pub_steering_5 = rospy.Publisher('/steering_motors/pid/motor5/setpoint', Float64, queue_size = 5)
pub_steering_6 = rospy.Publisher('/steering_motors/pid/motor6/setpoint', Float64, queue_size = 5)
pub_steering_7 = rospy.Publisher('/steering_motors/pid/motor7/setpoint', Float64, queue_size = 5)
pub_steering_8 = rospy.Publisher('/steering_motors/pid/motor8/setpoint', Float64, queue_size = 5)


# ~~~~~~~~~~~~~~~~~~~~~~ ROTATE WHEELS ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad_5_7 = 0.785
angle_rad_6_8 = -0.785
msg_5_7 = Float64()
msg_6_8 = Float64()
msg_5_7.data = angle_rad_5_7
msg_6_8.data = angle_rad_6_8

step1 = time.time() + 3
sending_command_interval = 0.1

last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_driving_1.publish(msg_driving)
        pub_driving_2.publish(msg_driving)
        pub_driving_3.publish(msg_driving)
        pub_driving_4.publish(msg_driving)

        pub_steering_5.publish(msg_5_7)
        pub_steering_6.publish(msg_6_8)
        pub_steering_7.publish(msg_5_7)
        pub_steering_8.publish(msg_6_8)

        last_time = now


    if now > step1:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ ROTATE ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms_1_4 = 0.8
speed_ms_2_3 = -0.8
msg_1_4 = Float64()
msg_2_3 = Float64()
msg_1_4.data = speed_ms_1_4
msg_2_3.data = speed_ms_2_3

angle_rad_5_7 = 0.785
angle_rad_6_8 = -0.785
msg_5_7 = Float64()
msg_6_8 = Float64()
msg_5_7.data = angle_rad_5_7
msg_6_8.data = angle_rad_6_8

step2 = time.time() + 27
sending_command_interval = 0.1

last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_driving_1.publish(msg_1_4)
        pub_driving_2.publish(msg_2_3)
        pub_driving_3.publish(msg_2_3)
        pub_driving_4.publish(msg_1_4)

        pub_steering_5.publish(msg_5_7)
        pub_steering_6.publish(msg_6_8)
        pub_steering_7.publish(msg_5_7)
        pub_steering_8.publish(msg_6_8)

        last_time = now


    if now > step2:
        break



# ~~~~~~~~~~~~~~~~~~~~~~ STOP ~~~~~~~~~~~~~~~~~~~~~~  
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0.0
msg_steering = Float64()
msg_steering.data = angle_rad


step3 = time.time() + 2
sending_command_interval = 0.1

last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_driving_1.publish(msg_driving)
        pub_driving_2.publish(msg_driving)
        pub_driving_3.publish(msg_driving)
        pub_driving_4.publish(msg_driving)

        pub_steering_5.publish(msg_steering)
        pub_steering_6.publish(msg_steering)
        pub_steering_7.publish(msg_steering)
        pub_steering_8.publish(msg_steering)

        last_time = now


    if now > step3:
        break

