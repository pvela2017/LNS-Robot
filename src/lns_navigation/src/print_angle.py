#!/usr/bin/env python3

"""
Print the current orientation of
the robot, to be used in rotation
in place navigation mode.


by Pablo
Last review: 2023/07/10
"""


import rospy
import tf
import geometry_msgs

from nav_msgs.msg import Odometry

def talker(msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        print(euler[2])


rospy.init_node('orientation')
rospy.Subscriber("/odometry/filtered", Odometry, talker, queue_size=1)




rospy.spin()