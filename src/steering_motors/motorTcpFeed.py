#!/usr/bin/env python3

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

"""
Code to read all 4 steering angles
through TCP-CAN converter

Send: ?0
Receive: 

Connect to socket 3

node: ggm_feedback
Publish: /steeringmotors/feedback 
Multi array [SA0, SA1, SA2, SA3]
SA: Steering Angle

by Pablo
Last review: 2022/09/24
"""

import rospy
import socket
import numpy as np

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from scripts.socketDic import socket3
from std_msgs.msg import Float64MultiArray

# Sock configuration
server_address = (socket3["HOST"], socket3["PORT"])


def recv(sock):
    # Check that there is a message in the buffer if it doesnt receive anything the timeout function
    # will raise an error
    try:
        # Do-while loop emulation in python
        data = sock.recv(15)
        #print(data)
        data_decoded = str(data).split("'")[1] # already dec no need to decode using bytes.hex(data, ',')
        #print(data_decoded)
        # Array received has the from: -----------------------
        # When splitting the array, the result is a string
        # Check that it is the postion feedback
        if int(data_decoded.split(",")[0]) != "!1":
            angle0 = data_decoded.split(",")[0]
            angle1 = data_decoded.split(",")[1]
            angle2 = data_decoded.split(",")[2]
            angle3 = data_decoded.split(",")[3]
            return [angle0, angle1, angle2, angle3]
        
        while True:
            data = sock.recv(15)
            #print(data)
            data_decoded = str(data).split("'")[1] # already dec no need to decode using bytes.hex(data, ',')
            #print(data_decoded)
            # Array received has the from: -----------------------
            # When splitting the array, the result is a string
            # Check that it is the postion feedback
            if int(data_decoded.split(",")[0]) != "!1":
                angle0 = data_decoded.split(",")[0]
                angle1 = data_decoded.split(",")[1]
                angle2 = data_decoded.split(",")[2]
                angle3 = data_decoded.split(",")[3]
                return [angle0, angle1, angle2, angle3]
        
    except:
        return [None, None, None, None]

def request():
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(0.01) # Set 0.01 seconds for socket timeout 

    wheels_angle = []

    sock.sendall("?0".encode()) # Send angle request
    # Receive data
    angle0, angle1, angle2, angle3 = recv(sock)

    # Check if there was no reply
    if angle0 != None: 
        # Fill values
        wheels_angle.append(angleScale(int(angle0)))
        wheels_angle.append(angleScale(int(angle1)))
        wheels_angle.append(angleScale(int(angle2)))
        wheels_angle.append(angleScale(int(angle3)))
        #print('Angle1:{}  Angle2:{}  Angle3:{}  Angle4:{}'.format(wheels_angle[0], wheels_angle[1], wheels_angle[2], wheels_angle[3]))
    else:
        print("No reply")


    # Form message
    ggm_feedback = Float64MultiArray()

    for i in wheels_angle:
        #print('Angle:{} '.format(i))
        ggm_feedback.data.append(i)

    # Close the socket
    #sock.shutdown(socket.SHUT_RDWR)
    #sock.close()

    return ggm_feedback


def angleScale(angle):
    """
    Function to scale the angles values between 0 to 180
    """
    if angle >= 180:
        angle-=270
    else:
        angle+=90

    return angle


def talker():
    rospy.init_node('steering_feedback')
    angles_pub = rospy.Publisher("/steeringmotors/feedback", Float64MultiArray, queue_size=1)
    rate = rospy.Rate(5) # 50hz
    msg = Float64MultiArray()

    while not rospy.is_shutdown():
        msg = request()
        # Check there was a reply before publishing
        if msg != None:
            angles_pub.publish(msg)
            
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

