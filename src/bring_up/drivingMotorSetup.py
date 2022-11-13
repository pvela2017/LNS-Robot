#!/usr/bin/env python3

"""
This scripts initialize the 4 driving MD750 motors
The parameters are:
        Velocity mode
        Inicial velocity 0
"""

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

import socket
import binascii

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from canDic import ggm
from socketDic import socket2

# Sock configuration
server_address = (socket2["HOST"], socket2["PORT"])


#                DLC               Fix    ID  
# setup_array = [0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

idMotor1 = 0x01
idMotor2 = 0x02
idMotor3 = 0x03
idMotor4 = 0x04

def checkRcv(sock, motorId, statusOK):
    # Check that there is a message in the buffer if it doesnt receive anything the timeout function
    # will raise an error
    try:
        data = sock.recv(13)
        data_decoded = bytes.hex(data, '-')
        #print(data_decoded)
        awk = data_decoded.split("-")[5]
        if awk == statusOK:
            return True
    except:
        #print("GGM Reply Timeout")
        return False


def motorSetup(sock, motorId):
    # Turn the brake ON   PID:#06 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0
    

    # Set inversion of moving direction (Some motors) PID:#16 [Read Write (No need for 0xaa)] 
    if motorId == 0x02 or motorId == 0x04:
        setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    else:
        setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0

    
    # Set speed = 0 then control zero speed, not braking  PID:#24 [Read Write (No need for 0xaa)] 
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0


    # Set operation mode: speed   PID:#183 [Read Write (No need for 0xaa)] 
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0xB7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0

    # Free mode on, brake OFF  PID:#05 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0

    # Close loop speed = 0  PID:#130 [Command]
    setup_array = [0x08, 0x00, 0x00, 0x00, motorId, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    #print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0

    return 1

        

if __name__ == '__main__':
    # Connection to socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket2["TIMEOUT"]) # Set 0.01 seconds for socket timeout 
    print("Setup Connected to CAN bus")

    # Setup each motor
    if motorSetup(sock, idMotor1):
        print(Fore.GREEN + "Motor 1 Setup OK")
    else:
        print(Fore.RED + "Motor 1 Error")

    if motorSetup(sock, idMotor2):
        print(Fore.GREEN + "Motor 2 Setup OK")
    else:
        print(Fore.RED + "Motor 2 Error")

    if motorSetup(sock, idMotor3):
        print(Fore.GREEN + "Motor 3 Setup OK")
    else:
        print(Fore.RED + "Motor 3 Error")

    if motorSetup(sock, idMotor4):
        print(Fore.GREEN + "Motor 4 Setup OK")
    else:
        print(Fore.RED + "Motor 4 Error")

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
