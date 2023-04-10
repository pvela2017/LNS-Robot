#!/usr/bin/env python3

"""
Change blabla
"""

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Int64MultiArray


def rpmToradsec(rpm):
        radsec = rpm*0.10472
        return radsec

def talker5(msg):
        new_msg = Float64()
        new_msg.data = msg.data
        translator5.publish(new_msg)

def talker6(msg):
        new_msg = Float64()
        new_msg.data = msg.data
        translator6.publish(new_msg)

def talker7(msg):
        new_msg = Float64()
        new_msg.data = msg.data
        translator7.publish(new_msg)

def talker8(msg):
        new_msg = Float64()
        new_msg.data = msg.data
        translator8.publish(new_msg)


def talker9(msg):
        new_msg1 = Float64()
        new_msg2 = Float64()
        new_msg3 = Float64()
        new_msg4 = Float64()
        new_msg1.data = rpmToradsec(msg.data[0])
        new_msg2.data = -rpmToradsec(msg.data[1])
        new_msg3.data = -rpmToradsec(msg.data[2])
        new_msg4.data = rpmToradsec(msg.data[3])
        translator1.publish(new_msg1)
        translator2.publish(new_msg2)
        translator3.publish(new_msg3)
        translator4.publish(new_msg4)


rospy.init_node('sim_controller_translator')
rospy.Subscriber("/steering_motors/pid/motor5/setpoint", Float64, talker5, queue_size=1)
rospy.Subscriber("/steering_motors/pid/motor6/setpoint", Float64, talker6, queue_size=1)
rospy.Subscriber("/steering_motors/pid/motor7/setpoint", Float64, talker7, queue_size=1)
rospy.Subscriber("/steering_motors/pid/motor8/setpoint", Float64, talker8, queue_size=1)
rospy.Subscriber("/driving_motors/commands", Int64MultiArray, talker9, queue_size=1)


translator5 = rospy.Publisher('/steering_fl_position_controller/command', Float64, queue_size=1)
translator6 = rospy.Publisher('/steering_fr_position_controller/command', Float64, queue_size=1)
translator7 = rospy.Publisher('/steering_br_position_controller/command', Float64, queue_size=1)
translator8 = rospy.Publisher('/steering_bl_position_controller/command', Float64, queue_size=1)

translator1 = rospy.Publisher('/wheel_fl_velocity_controller/command', Float64, queue_size=1)
translator2 = rospy.Publisher('/wheel_fr_velocity_controller/command', Float64, queue_size=1)
translator3 = rospy.Publisher('/wheel_br_velocity_controller/command', Float64, queue_size=1)
translator4 = rospy.Publisher('/wheel_bl_velocity_controller/command', Float64, queue_size=1)


rospy.spin()