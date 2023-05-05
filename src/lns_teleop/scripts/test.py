#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Float64



rospy.init_node('test')
motor_5 = rospy.Publisher('/steering_motors/pid/motor5/setpoint', Float64, queue_size=1)
motor_6 = rospy.Publisher('/steering_motors/pid/motor6/setpoint', Float64, queue_size=1)
motor_7 = rospy.Publisher('/steering_motors/pid/motor7/setpoint', Float64, queue_size=1)
motor_8 = rospy.Publisher('/steering_motors/pid/motor8/setpoint', Float64, queue_size=1)

msg = Float64()
msg = 0.0

while True:
    motor_5.publish(msg)
    motor_6.publish(msg)
    motor_7.publish(msg)
    motor_8.publish(msg)