#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 17:57:01 2022

@author: stefan
"""
import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import os

ScenePath = os.path.dirname(os.path.abspath(__file__))+'/../../Scenes/'
PressureDataPath = ScenePath + 'PressureData.txt'
MatrixPath = ScenePath + 'Matrix.txt'

SerialObj = serial.Serial('/dev/ttyACM0',115200,timeout=0.1)

print("Start")

cols = 4
rows = 2

fig = plt.figure()
fig.suptitle('Capacitive Sensor Data', fontsize=16)
im = plt.imshow(np.empty((rows,cols)),cmap='gray', vmin=0, vmax=200, animated=True)
Back = np.zeros((rows,cols))
Matrix = np.zeros((rows,cols))
Iteration = 0
WaitFrames = 20

def updatefig(*args):
    global Back, First, Iteration
    Line = SerialObj.readline()    
    #print(Line)    
    try:
        Decoded = Line.decode()
        SplitStr = Decoded.split(',')
        Floats = np.array([float(i) for i in SplitStr])        
        Pressures = Floats[:4]
        print("Pressures: {}".format(Pressures))
        np.savetxt(PressureDataPath,Pressures)
        
        if (len(Floats)>4):
            FloatsMuCa = Floats[4:]
            Matrix = np.reshape(FloatsMuCa,(rows,cols)) - Back        
            if Iteration == WaitFrames:
                Back = Matrix            
            
            
            # remapping to account for                     
            Matrix = np.array([[Matrix[0,0], Matrix[0,2],Matrix[0,1],Matrix[0,3]],[Matrix[1,0], Matrix[1,3],Matrix[1,2],Matrix[1,1]]])
            print("Raw Matrix")
            print(Matrix)        
            MeanSect1 = np.mean(Matrix[0,1:])
            MeanSect2 = np.mean(Matrix[1,1:])
            
            Matrix[0,1:] = Matrix[0,1:]-MeanSect1
            Matrix[1,1:] = Matrix[1,1:]-MeanSect2
            print("Normalized Matrix")
            print(Matrix)        
            
            np.savetxt(MatrixPath,Matrix)
            im.set_array(Matrix)        
        
        
    except Exception as ex:    
        print(ex)
        pass
    Iteration += 1
    return im,


ani = animation.FuncAnimation(fig, updatefig, interval=1, blit=True)

plt.show()