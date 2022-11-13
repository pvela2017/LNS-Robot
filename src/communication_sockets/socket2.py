#!/usr/bin/env python3

"""
Send the steering commands through TCP

Connect to socket 2

node: socket2
Subscribe to: /socket2/send
Publish to: -


by Pablo
Last review: 2022/09/25
"""

import rospy
import socket
import numpy as np

from scripts.socketDic import socket2
from std_msgs.msg import Int64MultiArray

# Sock configuration
server_address = (socket2["HOST"], socket2["PORT"])

def sender(msg):
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)

    # Send data
    message = bytearray()
    for i in msg:
        message.append(i)
    #print(repr(message))
    sock.sendall(message) # Send data

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()


def listener():
    rospy.init_node('socket2')
    rospy.Subscriber("/socket2/send", Int64MultiArray, sender, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
 listener()
