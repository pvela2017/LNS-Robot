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

from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


class motores:
    def __init__(self):
        rospy.init_node('remap_fb_node_driving')
        self.motor1_ = 0
        self.motor2_ = 0
        self.motor3_ = 0
        self.motor4_ = 0

        self.command = Int64MultiArray()

        self.wheel_1_sub_ = rospy.Subscriber('/driving_pid/pid/motor1/control_effort', Float64, self.m1CB, queue_size=1)
        self.wheel_2_sub_ = rospy.Subscriber('/driving_pid/pid/motor2/control_effort', Float64, self.m2CB, queue_size=1)
        self.wheel_3_sub_ = rospy.Subscriber('/driving_pid/pid/motor3/control_effort', Float64, self.m3CB, queue_size=1)
        self.wheel_4_sub_ = rospy.Subscriber('/driving_pid/pid/motor4/control_effort', Float64, self.m4CB, queue_size=1)

        self.driving_motors_ = rospy.Publisher("/driving_motors/commands", Int64MultiArray, queue_size=1)


    def m1CB(self, msg):
        self.motor1_ = msg.data

    def m2CB(self, msg):
        self.motor2_ = msg.data

    def m3CB(self, msg):
        self.motor3_ = msg.data

    def m4CB(self, msg):
        self.motor4_ = msg.data


    def publishCommand(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.command.data.clear()
            self.command.data.append(int(self.motor1_))
            self.command.data.append(int(self.motor2_))
            self.command.data.append(int(self.motor3_))
            self.command.data.append(int(self.motor4_))

            self.driving_motors_.publish(self.command)

            rate.sleep()



if __name__ == '__main__':
    motores_ = motores()
    motores_.publishCommand()
