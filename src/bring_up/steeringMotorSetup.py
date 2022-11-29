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
    # Watchdog STOP
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x2B, 0x04, 0x50, 0x06, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.01)


    # Operation Mode: Position Mode
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.01)


    # Speed Protocol
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x81, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.01)


    # Acceleration Protocol
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x83, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.005)


    # Desacceleration Protocol
    setup_array = [0x08, 0x00, 0x00, 0x06, motorId, 0x23, 0x84, 0x60, 0x00, 0x20, 0xA1, 0x07, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.01)


    return 1

        

if __name__ == '__main__':
    # Connection to socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 
    print("Setup Connected to CAN bus")

    # NMT Operation
    setup_array = [0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    # Reply not working

    # Pause before next command or the controller cant proccess so much data
    time.sleep(0.01)

    motorSetup(sock, idMotor5)
    motorSetup(sock, idMotor6)
    motorSetup(sock, idMotor7)
    motorSetup(sock, idMotor8)

    print(Fore.GREEN + "ALL Motor Setup OK")

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

