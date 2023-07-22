#!/usr/bin/env python3

"""

Ackermann

CTRL-C to quit

by Pablo
Last review: 2022/07/26


"""

import time
import rospy
from geometry_msgs.msg import Twist



# Create node
rospy.init_node('demo')
odom = rospy.Publisher('/swerve_controller/cmd_vel', Twist, queue_size = 5)


# ~~~~~~~~~~~~~~~~~~~~~~ ROTATE ~~~~~~~~~~~~~~~~~~~~~~ 
# Setup
speed_x = 0.1
speed_z = -0.1
msg = Twist()
msg.linear.x = speed_x
msg.angular.z = speed_z


step1 = time.time() + 50
sending_command_interval = 0.1

last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        odom.publish(msg)
        last_time = now


    if now > step1:
        break



# ~~~~~~~~~~~~~~~~~~~~~~ STOP ~~~~~~~~~~~~~~~~~~~~~~  
# Setup
speed_x = 0.0
speed_z = 0.0
msg = Twist()
msg.linear.x = speed_x
msg.angular.z = speed_z


step2 = time.time() + 2
sending_command_interval = 0.1

last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        odom.publish(msg)

        last_time = now


    if now > step2:
        break

