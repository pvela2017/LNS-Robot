#!/usr/bin/env python3

"""

Circle

CTRL-C to quit

by Pablo
Last review: 2022/07/26
"""

import time
import socket
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray

from scripts.socketDic import socket2
from scripts.robotDic import robot

# Sock configuration
server_address = (socket2["HOST"], socket2["PORT"])


idMotors = [0x05, 0x06, 0x07, 0x08]

def angle2Bytes(angle, motorid):
    #print(angle)
    scaled_angle = int(abs(angle*1388))

    if (motorid == 0x05) or (motorid == 0x07):
        if angle < 0:
            dec = 4294967295 - scaled_angle
            byte0 = (dec >> 24) & 0xff
            byte1 = (dec >> 16) & 0xff
            byte2 = (dec >> 8) & 0xff
            byte3 = dec & 0xff
            return [byte3, byte2, byte1, byte0]

        if angle > 0:
            dec = scaled_angle
            byte0 = (dec >> 24) & 0xff
            byte1 = (dec >> 16) & 0xff
            byte2 = (dec >> 8) & 0xff
            byte3 = dec & 0xff
            return [byte3, byte2, byte1, byte0]

    if (motorid == 0x06) or (motorid == 0x08):
        if angle > 0:
            dec = 4294967295 - scaled_angle
            byte0 = (dec >> 24) & 0xff
            byte1 = (dec >> 16) & 0xff
            byte2 = (dec >> 8) & 0xff
            byte3 = dec & 0xff
            return [byte3, byte2, byte1, byte0]

        if angle < 0:
            dec = scaled_angle
            byte0 = (dec >> 24) & 0xff
            byte1 = (dec >> 16) & 0xff
            byte2 = (dec >> 8) & 0xff
            byte3 = dec & 0xff
            return [byte3, byte2, byte1, byte0]
    
    return [None, None, None, None]


def kinematics(angle):
    rad = np.radians(angle)

    print("taget angle:%s[deg], %s[rad]"%(angle, rad))
    """
    fl: front left wheel (id 5)
    rl: rear left wheel (id 8)
    fr: front right wheel (id 6)
    rr: rear right wheel (id 7)
    """

    L = robot["WHEEL_BASE_DISTANCE"]
    d = robot["AXLE_TRACK_DISTANCE"]
    print("L:%s, d:%s"%(L, d))

    fl = int(np.degrees(np.arctan(L/((L/np.tan(rad)) - d))))
    rl = - fl
    fr = int(np.degrees(np.arctan(L/((L/np.tan(rad)) + d))))
    rr = - fr
    # Return angles
    return [fl, fr, rr, rl]



# Create node
rospy.init_node('demo')
pub_dmke = rospy.Publisher('/drivingmotors/commands', Float64MultiArray)

# Change angle
# Connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)
sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

angle_kin = kinematics(15)
print(angle_kin)


for  motorId in idMotors:
    if (motorId == 0x08):
        angle = angle_kin[3]

    elif (motorId == 0x07):
        angle = angle_kin[2]

    elif (motorId == 0x06):
        angle = angle_kin[1]
    else:
        angle = angle_kin[0]

    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(angle, motorId)

    if setup_array[9] != None:
        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause
        time.sleep(0.01)  

        # Set to relative motion
        setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)

        # Execute Positioning
        setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)     


dmke_msg = Float64MultiArray()
speed = 0.40
dmke_msg.data = [speed, speed, speed, speed]
pub_dmke.publish(dmke_msg)

timeout = time.time() + 42
sending_command_interval = 0.005


# Doing the circle
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > timeout:
        break

# STOP
speed = 0.0
dmke_msg.data = [speed, speed, speed, speed]
pub_dmke.publish(dmke_msg)
