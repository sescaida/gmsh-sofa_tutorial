import numpy as np
import threading
from multiprocessing import Process
import serial
import os

ScenePath = os.path.dirname(os.path.abspath(__file__))+'/../../'
#from keras.models import model_from_json

class FuncThread(threading.Thread):
    def __init__(self,t,*a):
        self._t=t
        self._a=a
        threading.Thread.__init__(self)
    def run(self):
        self._t(*self._a)

class PressureSensorsApp():    

    def __init__(self, PortName, Plot2DPos=False):
        # Path for saving current reading for data exchanges with sofa
        self.PressureDataFilePath = ScenePath + 'PressureData.txt'
        self.OldValues = [0,0,0,0,0]
        self.SerialObj = serial.Serial("/dev/"+ PortName,115200)
        # running stuff
        self.PauseRunning = False
        self.StopRunning = False
        self.Plotting = False  

        
        self.ExceptionMaxCount = 10
        self.ExceptionCounter = 0     
        
        self.ReadoutThread = FuncThread(self.updateBuffers)
        
        self.ReadoutThread.start()
        
    def readFromSerial(self, SerialObj):
        Res = ''        
        try:
            Res = SerialObj.readline()            
            return Res
        except: # SerialError as e:
            print('Error reading from serial: ' + SerialObj.port)                        
            print(self.StopRunning)
            self.ExceptionCounter = self.ExceptionCounter + 1
            if (self.ExceptionCounter > self.ExceptionMaxCount):
                print('stopping readout due to (to many) errors')
                self.stop()
                return            
            
    def updateBuffers(self):    
        
        while(True):
            try:
                self.CurrentValuesAsStr = self.readFromSerial(self.SerialObj)            
                            
                try:                
                    StringObj = self.CurrentValuesAsStr.decode()            
                    Val1,Val2,Val3,Val4,Val5 = StringObj.split(',')                        
                    self.CurrentValues = [float(Val1), float(Val2), float(Val3), float(Val4),int(Val5)] 
                    print("CurrentValues: ", self.CurrentValues)                             
                    self.OldValues = self.CurrentValues            
                except:
                    print('Warning: could not convert the following string: ' + StringObj)
                    self.CurrentValues = self.OldValues          
                np.savetxt(self.PressureDataFilePath, self.CurrentValues)                           
            except:
                print("ouch!")
                pass
        
MyPressureSensors = PressureSensorsApp("ttyACM0", False)