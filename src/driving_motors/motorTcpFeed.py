#!/usr/bin/env python3

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

"""
Code to setup positive RPMs

through TCP-CAN converter

by Pablo
Last review: 2022/07/08
"""

import rospy
import socket
import numpy as np

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from scripts.socketDic import socket1
from scripts.robotDic import robot
from std_msgs.msg import Float64MultiArray

# Sock configuration
server_address = (socket1["HOST"], socket1["PORT"])

# Array setup for writting velocity
#                                   MotorID  READ  SPEED                         
setup_array = [0x08, 0x00, 0x00, 0x00, 0x01, 0x04, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


idMotors = [0x01, 0x02, 0x03, 0x04]


def byteTorpm(byte0, byte1):
    # Transform the bytes received into the motor rpm
    conc_number = byte1 + byte0
    # print(conc_number)
    dec = int(conc_number, 16)
    # print(dec)
  
    # Positive rpm
    if dec <= 32767:  
        motor_rpm = dec

    # Negative rpm
    # http://www.technosoft.ro/KB/index.php?/article/AA-15440/0/Negative-numbers-representation-in-hex.html
    else:
        pre_motor_rpm = 65535 - dec # FFFF - dec
        motor_rpm = -1*pre_motor_rpm

    return motor_rpm

def rpmTovel(motor_rpm):
    # Transform from motor rpm to m/s linear velocity
    real_rpm = motor_rpm/robot["DRIVING_GEAR_BOX_RATIO"] # DRIVING_GEAR_BOX_RATIO from robot dic 
    vel = real_rpm*(2*robot["WHEELS_RADIUS"]*np.pi)/60  # WHEELS_RADIUS from robot dic  
    return vel

def recv(sock, motorId):
    # Check that there is a message in the buffer if it doesnt receive anything the timeout function
    # will raise an error
    try:
        # Do-while loop emulation in python
        data = sock.recv(13)
        data_decoded = bytes.hex(data, '-')
        #print(data_decoded)
        # When splitting the bytes, the result is a string
        # Check that it is the velocity feedback
        if int(data_decoded.split("-")[5], 16) == 138 and True:
            byte0 = data_decoded.split("-")[6]
            byte1 = data_decoded.split("-")[7]
            return [byte0, byte1]
        
        while True:
            data = sock.recv(13)
            data_decoded = bytes.hex(data, '-')
            #print(data_decoded)
            # When splitting the bytes, the result is a string
            # Check that it is the velocity feedback
            if int(data_decoded.split("-")[5], 16) == 138 and True:
                byte0 = data_decoded.split("-")[6]
                byte1 = data_decoded.split("-")[7]
                return [byte0, byte1]
        
    except:
        return [None, None]

def request():
    # Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(0.01) # Set 0.01 seconds for socket timeout 

    motor_rpm = []
    wheels_rpm = []

    # Motor
    for motors in idMotors:
        setup_array[4] = motors
    
        # Send data
        message = bytearray()
        for i in setup_array:
            message.append(i)
        #print(repr(message))
        sock.sendall(message) # Send data

        # Receive data
        byte0, byte1 = recv(sock, setup_array[3])

        # Check if there was no reply
        if byte0 != None: 
            # Process data
            motor_rpm.append(byteTorpm(byte0, byte1))
            wheels_rpm.append(rpmTovel(motor_rpm[-1]))
        #else:
            #print("No reply")

    # Form message
    feedback = Float64MultiArray()

    for i,j in zip(motor_rpm, wheels_rpm):
        #print('Motor speed:{}  Wheel speed:{}'.format(i, j))
        feedback.data.append(i)
        feedback.data.append(j)

    return feedback

    # Close the socket
    #sock.shutdown(socket.SHUT_RDWR)
    #sock.close()




def talker():
    rospy.init_node('driving_feedback')
    speeds_pub = rospy.Publisher("/drivingmotors/feedbacks", Float64MultiArray, queue_size=1)
    rate = rospy.Rate(12) # 50hz
    msg = Float64MultiArray()

    while not rospy.is_shutdown():
        msg = request()
        print(msg.data[1])
        speeds_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

