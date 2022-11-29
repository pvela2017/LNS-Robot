#!/usr/bin/env python3

"""
This scripts initialize the 4 steering motors
The parameters are:


"""

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

import rospy

import socket
import binascii
import numpy as np

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from scripts.canDic import ggm
from scripts.socketDic import socket1
from scripts.robotDic import robot
from std_msgs.msg import Float64MultiArray

# Sock configuration
server_address = (socket1["HOST"], socket1["PORT"])


#                DLC             Fix    ID  
setup_array = [0x08, 0x00, 0x00, 0x00, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

idMotors = [0x01, 0x02, 0x03, 0x04]

def rpmTobyte(rpm):
    # Transform the rpm value into bytes to be send
    dec = int(rpm)
    # If rpm are positive
    if dec >= 0:
        # 16 bites = 8bites0 8bites1 
        byte0 = (dec >> 8) & 0xff
        byte1 = dec & 0xff
        # But they need to be reversed
        # byte0 is the less significative bits
        # byte1 is the more significative bits
        # Thus right order is
        return [byte1, byte0]
    # If rpm are negative
    else:
        neg_dec = 65535 - abs(dec) # FFFF - rpm
        # 16 bites = 8bites0 8bites1 
        byte0 = (dec >> 8) & 0xff
        byte1 = dec & 0xff
        # But they need to be reversed
        # byte0 is the less significative bits
        # byte1 is the more significative bits
        # Thus right order is
        return [byte1, byte0]

def velTorpm(vel):
    # Check for maximum speed
    if abs(vel) > robot["MAX_LINEAR_SPEED"]:
        sign_vel = np.sign(vel)
        vel = sign_vel*robot["MAX_LINEAR_SPEED"]
    # Transform from m/s linear velocity to motor rpm
    rpm = (vel*60)/(2*robot["WHEELS_RADIUS"]*np.pi) # WHEELS_RADIUS from robot dic
    real_rpm = rpm*robot["DRIVING_GEAR_BOX_RATIO"] # DRIVING_GEAR_BOX_RATIO from robot dic
    return real_rpm

def comm(msg):
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket1["TIMEOUT"]) # Set 0.01 seconds for socket timeout 

    # Loop between all the motors
    for motor_speed, motorId in zip(msg.data, idMotors):
        # Unpack msg
        vel_ms = motor_speed

        # Data setup
        setup_array[4] = motorId
        setup_array[6], setup_array[7] = rpmTobyte(velTorpm(vel_ms))

        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data


def sender_shutdown(msg):
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)

    # Send data
    message = bytearray()
    for i in msg:
        message.append(int(i))
    #print(repr(message))
    sock.sendall(message) # Send data

    #sock.shutdown(socket.SHUT_RDWR)
    #sock.close()

def motor_stop():
    array = [0x08, 0x00, 0x00, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    sender_shutdown(array)

    array = [0x08, 0x00, 0x00, 0x00, 0x02, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    sender_shutdown(array)

    array = [0x08, 0x00, 0x00, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    sender_shutdown(array)

    array = [0x08, 0x00, 0x00, 0x00, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    sender_shutdown(array)


def listener():
    rospy.init_node('driving_commands')
    rospy.on_shutdown(motor_stop)
    rospy.Subscriber("/drivingmotors/commands", Float64MultiArray, comm, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
 listener()
