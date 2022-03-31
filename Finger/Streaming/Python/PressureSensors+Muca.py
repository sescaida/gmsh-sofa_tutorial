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
MuCaDataPath = ScenePath + 'MuCaData.txt'

SerialObj = serial.Serial('/dev/ttyACM0',115200,timeout=0.1)

print("Start")

cols = 4
rows = 2

fig = plt.figure()
im = plt.imshow(np.empty((rows,cols)),cmap='gray', vmin=0, vmax=700, animated=True)
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
        np.savetxt(PressureDataPath,Pressures)
        
        if (len(Floats)>4):
            FloatsMuCa = Floats[4:]
            Matrix = np.reshape(FloatsMuCa,(rows,cols)) - Back        
            if Iteration == WaitFrames:
                Back = Matrix            
            print(Matrix)        
            np.savetxt(MuCaDataPath,Matrix)
            im.set_array(Matrix)        
        
        
    except Exception as ex:    
        print(ex)
        pass
    Iteration += 1
    return im,


ani = animation.FuncAnimation(fig, updatefig, interval=1, blit=True)

plt.show()