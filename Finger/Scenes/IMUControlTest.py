#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 14 19:01:29 2022

@author: stefan
"""

#import time
import serial
import numpy as np
FileMotorCommands = "MotorCommands.txt"
FileIMUData = "IMUData.txt"
SerialObj = serial.Serial("/dev/ttyACM0", 115200,timeout=0.1)
OldMotorValues = np.zeros(3, dtype=int)
DesiredMotorValues = OldMotorValues
while True:
    CurrentLine = SerialObj.readline()   
    try:
        DecodedAndSplit = CurrentLine.decode().split(',')
        FloatValues = []
        for String in DecodedAndSplit[:4]:
            FloatValues.append(float(String))
    
        #print("Current IMU values (quaternion): {}".format(FloatValues))        
        np.savetxt(FileIMUData, FloatValues)
        
    except:
        print("Error while decoding/writing IMU data")
    
    
    try:
        DesiredMotorValues = np.loadtxt(FileMotorCommands,dtype=int)       
        if len(DesiredMotorValues) == 0:
            print("Warning: read emtpy motor command!")
            continue
    except Exception as inst:
        print(inst.__str__())
        print("Error while reading motor command")
        continue
        
#    print("OldMotorValues: {}".format(OldMotorValues))
#    print("DesiredMotorValues: {}".format(DesiredMotorValues))
    if any(OldMotorValues!=DesiredMotorValues):
        String = str(DesiredMotorValues[0]) + ' ' + str(DesiredMotorValues[1]) + ' ' + str(DesiredMotorValues[2]) + '\n'
        ByteString = String.encode('ASCII')
        print("Sending to the motors: {}".format(ByteString))
        SerialObj.write(ByteString)
        OldMotorValues = DesiredMotorValues
    
    