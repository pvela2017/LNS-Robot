#!/usr/bin/env python3

"""
This scripts initialize the 4 steering motors
The parameters are:


"""

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

import time

import socket
import binascii

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from scripts.canDic import ggm
from scripts.socketDic import socket2

# Sock configuration
server_address = (socket2["HOST"], socket2["PORT"])


#                DLC               Fix    ID  
# setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x2B, 0x04, 0x50, 0x06, 0x00, 0x00, 0x00, 0x00]

idMotor5 = 0x05
idMotor6 = 0x06
idMotor7 = 0x07
idMotor8 = 0x08

def checkRcv(sock, motorId, statusOK):
    # Check that there is a message in the buffer if it doesnt receive anything the timeout function
    # will raise an error
    try:
        data = sock.recv(13)
        data_decoded = bytes.hex(data, '-')
        print(data_decoded)
        awk = data_decoded.split("-")[6]
        if awk == statusOK:
            return True
    except:
        print("Reply Timeout")
        return False


def motorSetup(sock, motorId):
    # Turn the brake ON   PID:#06 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)


    # Set number of poles   PID:#21 [Read Write (No need for 0xaa)] 
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0
    # Pause before next command or the controller cant proccess so much data
    rospy.sleep(0.05)
    

    # Set encoder CPR   PID:#156 [Read Write (No need for 0xaa)] 
    # There are no encoders

    # Set inversion of moving direction (Some motors) PID:#16 [Read Write (No need for 0xaa)] 
    if motorId == 0x02 or motorId == 0x03:
        setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    else:
        setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)


    # Set speed = 0 then control zero speed, not braking  PID:#24 [Read Write (No need for 0xaa)] 
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)


    # Set operation mode: speed   PID:#183 [Read Write (No need for 0xaa)] 
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0xB7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working
    
    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)
    

    # Free mode on, brake OFF  PID:#05 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working
    
    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)


    # Close loop speed = 0  PID:#130 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working


    return 1

        

if __name__ == '__main__':
    # Connection to socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 
    print("Setup Connected to CAN bus")

    motorSetup(sock, idMotor5)
    motorSetup(sock, idMotor6)
    motorSetup(sock, idMotor7)
    motorSetup(sock, idMotor8)

    print(Fore.GREEN + "ALL Motor Setup OK")

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

