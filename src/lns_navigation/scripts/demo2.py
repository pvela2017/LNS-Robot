#!/usr/bin/env python3

"""

Diagonal

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

angle_rad = 0.785
msg_steering = Float64()
msg_steering.data = angle_rad

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

        pub_steering_5.publish(msg_steering)
        pub_steering_6.publish(msg_steering)
        pub_steering_7.publish(msg_steering)
        pub_steering_8.publish(msg_steering)

        last_time = now


    if now > step1:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ RIGHT ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = 0.4
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0.785
msg_steering = Float64()
msg_steering.data = angle_rad

step2 = time.time() + 20
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


# ~~~~~~~~~~~~~~~~~~~~~~ ROTATE WHEELS ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = -0.785
msg_steering = Float64()
msg_steering.data = angle_rad

step4 = time.time() + 3
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


    if now > step4:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ LEFT ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = 0.4
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = -0.785
msg_steering = Float64()
msg_steering.data = angle_rad

step5 = time.time() + 20
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


    if now > step5:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ STOP ~~~~~~~~~~~~~~~~~~~~~~  
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0.0
msg_steering = Float64()
msg_steering.data = angle_rad


step6 = time.time() + 2
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


    if now > step6:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ ROTATE WHEELS ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0.0
msg_steering = Float64()
msg_steering.data = angle_rad

step7 = time.time() + 3
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


    if now > step7:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ BACK ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_ms = -0.4
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0
msg_steering = Float64()
msg_steering.data = angle_rad

step8 = time.time() + 30
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


    if now > step8:
        break


# ~~~~~~~~~~~~~~~~~~~~~~ STOP ~~~~~~~~~~~~~~~~~~~~~~  
# Setup
speed_ms = 0.0
msg_driving = Float64()
msg_driving.data = speed_ms

angle_rad = 0.0
msg_steering = Float64()
msg_steering.data = angle_rad


step9 = time.time() + 2
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


    if now > step9:
        break