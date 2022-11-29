#!/usr/bin/env python3

"""

Circle

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


idMotor5 = 0x05
idMotor6 = 0x06
idMotor7 = 0x07
idMotor8 = 0x08


def angle2Bytes(angle, motorid):
    #print(angle)
    scaled_angle = int(abs(angle*1388))

    if (motorid == idMotor5) or (motorid == idMotor7):
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

    if (motorid == idMotor6) or (motorid == idMotor8):
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
        



def comm(msg):
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

    # Motor 5 zeroing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    angle1 = msg.data[0]

    setup_array = [0x08, 0x00, 0x00, 0x06, idMotor5, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(angle1 - 90, idMotor5)

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
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor5, 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)

        # Execute Positioning
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor5, 0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)     

    
    # Motor 6 zeroing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    angle2 = msg.data[1]

    setup_array = [0x08, 0x00, 0x00, 0x06, idMotor6, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(angle2 - 90, idMotor6)

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
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor6, 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)

        # Execute Positioning
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor6, 0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01) 



    # Motor 7 zeroing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    angle3 = msg.data[2]

    setup_array = [0x08, 0x00, 0x00, 0x06, idMotor7, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(angle3 - 90, idMotor7)

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
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor7, 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)

        # Execute Positioning
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor7, 0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)


    # Motor 8 zeroing ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    angle4 = msg.data[3]

    setup_array = [0x08, 0x00, 0x00, 0x06, idMotor8, 0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    setup_array[9], setup_array[10], setup_array[11], setup_array[12] = angle2Bytes(angle4 - 90, idMotor8)

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
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor8, 0x2B, 0x40, 0x60, 0x00, 0x4F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01)

        # Execute Positioning
        setup_array = [0x08, 0x00, 0x00, 0x06, idMotor8, 0x2B, 0x40, 0x60, 0x00, 0x5F, 0x00, 0x00, 0x00]

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Pause for the controller
        time.sleep(0.01) 
    

def listener():
    rospy.init_node('zeroing')
    rospy.Subscriber("/steeringmotors/feedback", Float64MultiArray, comm, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
 listener()