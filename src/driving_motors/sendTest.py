#!/usr/bin/env python3

"""
This scripts initialize the 4 steering motors
The parameters are:


"""

# debug sudo tcpflow -i any -C -g port <port>   muestra el flujo entrante y saliente en el puerto
#       nmap <ip>                               entrega los puertos abiertas en la ip

import socket
import binascii

import colorama
from colorama import Fore
from colorama import init
init(autoreset=True) # reset color to default

from scripts.canDic import ggm
from scripts.socketDic import socket1

# Sock configuration
server_address = (socket1["HOST"], socket1["PORT"])


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
    # Free mode on, brake OFF  PID:#05 [Command]
    # Array setup for writting velocity
    #                             MotorID  READ  SPEED                         
    setup_array = [0x00, 0x00, 0x00, 0x01, 0x04, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    message = bytearray()
    for i in setup_array:
        message.append(i)
    sock.sendall(message) # Send data
    print(repr(message))
    if not checkRcv(sock, motorId, ggm["STATUS_OK"]):
        return 0

    return 1

        

if __name__ == '__main__':
    # Connection to socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    sock.settimeout(socket1["TIMEOUT"]) # Set 0.01 seconds for socket timeout 
    print("Setup Connected to CAN bus")

    # Setup each motor
    if motorSetup(sock, idMotor4):
        print(Fore.GREEN + "GGM Motor Setup OK")
    else:
        print(Fore.RED + "GGM Motor Error")


    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
