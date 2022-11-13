#!/usr/bin/env python3


"""
Transform the messaage from the steering commands topic
to a float message that the PID library can suscribe to.

node: remap_sp_node
Subscribe to: /steeringmotors/commands
Publish to :  /wheel_1_steering_pid/setpoint
              /wheel_2_steering_pid/setpoint
              /wheel_3_steering_pid/setpoint
              /wheel_4_steering_pid/setpoint


by Pablo
Last review: 2022/07/29
"""

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


def talker(msg):
    sp1_msg = Float64()
    sp2_msg = Float64()
    sp3_msg = Float64()
    sp4_msg = Float64()

    sp1_msg.data = msg.data[0]
    sp2_msg.data = msg.data[1]
    sp3_msg.data = msg.data[2]
    sp4_msg.data = msg.data[3]

    sp_wheel_1_pub.publish(sp1_msg)
    sp_wheel_2_pub.publish(sp2_msg)
    sp_wheel_3_pub.publish(sp3_msg)
    sp_wheel_4_pub.publish(sp4_msg)



rospy.init_node('remap_sp_node')
rospy.Subscriber("/steeringmotors/commands", Float64MultiArray, talker, queue_size=1)

sp_wheel_1_pub = rospy.Publisher('/wheel_1_steering_pid/setpoint', Float64, queue_size=1)
sp_wheel_2_pub = rospy.Publisher('/wheel_2_steering_pid/setpoint', Float64, queue_size=1)
sp_wheel_3_pub = rospy.Publisher('/wheel_3_steering_pid/setpoint', Float64, queue_size=1)
sp_wheel_4_pub = rospy.Publisher('/wheel_4_steering_pid/setpoint', Float64, queue_size=1)

rospy.spin()

