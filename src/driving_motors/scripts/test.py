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

from std_msgs.msg import Float64MultiArray

# Sock configuration
server_address = ("192.168.0.7", 20001)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(server_address)
sock.settimeout(0.01) # Set 0.01 seconds for socket timeout

print("connected")

setup_array = [0x08, 0x00, 0x00, 0x00, 0x03, 0x86, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
message = bytearray()
for i in setup_array:
    message.append(i)
sock.sendall(message) # Send data



