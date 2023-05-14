#!/usr/bin/env python3

"""
Change blabla
"""

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped

def talker(msg):
        heading = Imu()
        heading.header = msg.header
        # GPS orientation rotation is clockwise it should be counter clockwise
        # https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html#:~:text=Inverting%20or%20conjugating%20a%20rotation,it%20to%20its%20original%20location.
        heading.orientation.x = msg.quaternion.x
        heading.orientation.y = msg.quaternion.y
        heading.orientation.z = msg.quaternion.z
        heading.orientation.w = -msg.quaternion.w

        # Dummy data
        heading.angular_velocity.x = 0
        heading.angular_velocity.y = 0
        heading.angular_velocity.z = 0

        heading.linear_acceleration.x = 0
        heading.linear_acceleration.y = 0
        heading.linear_acceleration.z = 0

        translator.publish(heading)


rospy.init_node('heading_translator')
rospy.Subscriber("/heading", QuaternionStamped, talker, queue_size=1)

translator = rospy.Publisher('/heading_rl', Imu, queue_size=1)


rospy.spin()