#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  4 15:52:38 2022

@author: sibl
"""

def rpmTobyte(rpm):
    # If rpm are positive
    if rpm >= 0:
        # 16 bytes = 8bytes0 8bytes1 
        byte0 = (rpm >> 8) & 0xff
        byte1 = rpm & 0xff
        # But they need to be reversed
        # byte0 is the less significative bits
        # byte1 is the more significative bits
        # Thus right order is
        return [byte1, byte0]
    # If rpm are negative
    else:
        neg_dec = 65535 - abs(rpm) # FFFF- rpm
        # 16 bytes = 8bytes0 8bytes1 
        byte0 = (neg_dec >> 8) & 0xff
        byte1 = neg_dec & 0xff
        # But they need to be reversed
        # byte0 is the less significative bits
        # byte1 is the more significative bits
        # Thus right order is
        return [byte1, byte0]
    
    

print(rpmTobyte(-500))