#!/usr/bin/env python3

"""

Diagonal

CTRL-C to quit

by Pablo
Last review: 2022/07/26
"""

import time
import socket

import rospy
from std_msgs.msg import Float64MultiArray

from scripts.socketDic import socket2

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


# Create node
rospy.init_node('demo')
pub_dmke = rospy.Publisher('/drivingmotors/commands', Float64MultiArray)

dmke_msg = Float64MultiArray()
speed = 0.70
dmke_msg.data = [speed, speed, speed, speed]

step1 = time.time() + 10
sending_command_interval = 0.005

# rotate here
# rotate here
# Connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)
sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

for  motorId in idMotors:
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(45, motorId)

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

# Going DIAGONAL RIGHT
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > step1:
        break

# STOP and rotate wheels
speed = 0.0
dmke_msg.data = [speed, speed, speed, speed]
pub_dmke.publish(dmke_msg)

# rotate here
# Connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)
sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

for  motorId in idMotors:
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(-90, motorId)

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

step2 = time.time() + 3
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > step2:
        break


# Going DIAGONAL LEFT
speed = 0.70
dmke_msg.data = [speed, speed, speed, speed] # Change this!


step3 = time.time() + 10
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > step3:
        break



# STOP and rotate wheels
speed = 0.0
dmke_msg.data = [speed, speed, speed, speed]
pub_dmke.publish(dmke_msg)

# rotate here
# rotate here
# Connection
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)
sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

for  motorId in idMotors:
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(45, motorId)

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

step4 = time.time() + 3
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > step4:
        break



# Going BACKWARD
speed = -0.70
dmke_msg.data = [speed, speed, speed, speed]


step5 = time.time() + 17
last_time = time.time()
while True:
    now = time.time()

    if (now - last_time) >= sending_command_interval:
        pub_dmke.publish(dmke_msg)
        last_time = now


    if now > step5:
        break


# STOP
speed = 0.0
dmke_msg.data = [speed, speed, speed, speed]
pub_dmke.publish(dmke_msg)