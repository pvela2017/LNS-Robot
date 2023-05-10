#!/usr/bin/env python3


"""
Transform the messaage from the steering feedbaack topic
to a float message that the PID library can suscribe to.

node: remap_fb_node
Subscribe to: /steeringmotors/feedback
Publish to :  /wheel_1_steering_pid/state
              /wheel_2_steering_pid/state
              /wheel_3_steering_pid/state
              /wheel_4_steering_pid/state


by Pablo
Last review: 2022/09/29
"""

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

def radTorpm(radsec):
    wheel_rpm = (radsec *60.0)/(2.0*3.14)
    return wheel_rpm;

def talker(msg):
    fb1_msg = Float64()
    fb2_msg = Float64()
    fb3_msg = Float64()
    fb4_msg = Float64()

    fb1_msg.data = radTorpm(msg.data[0])
    fb2_msg.data = -radTorpm(msg.data[1])
    fb3_msg.data = radTorpm(msg.data[2])
    fb4_msg.data = -radTorpm(msg.data[3])

    fb_wheel_1_pub.publish(fb1_msg)
    fb_wheel_2_pub.publish(fb2_msg)
    fb_wheel_3_pub.publish(fb3_msg)
    fb_wheel_4_pub.publish(fb4_msg)



rospy.init_node('remap_fb_node_driving')
rospy.Subscriber("/driving_motors/feedback/angular/radsec", Float64MultiArray, talker, queue_size=1)

fb_wheel_1_pub = rospy.Publisher('/driving_pid/pid/motor1/state', Float64, queue_size=1)
fb_wheel_2_pub = rospy.Publisher('/driving_pid/pid/motor2/state', Float64, queue_size=1)
fb_wheel_3_pub = rospy.Publisher('/driving_pid/pid/motor3/state', Float64, queue_size=1)
fb_wheel_4_pub = rospy.Publisher('/driving_pid/pid/motor4/state', Float64, queue_size=1)

rospy.spin()

