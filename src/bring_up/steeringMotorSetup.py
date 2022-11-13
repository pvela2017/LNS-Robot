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

from canDic import ggm
from socketDic import socket2

# Sock configuration
server_address = (socket2["HOST"], socket2["PORT"])


idMotor1 = 0x01
idMotor2 = 0x02
idMotor3 = 0x03
idMotor4 = 0x04

