import numpy as np
import threading
from multiprocessing import Process
import serial
import os
import time
import SignalBufferTimeFlow
import visdom


ScenePath = os.path.dirname(os.path.abspath(__file__))+'/../../'

class FuncThread(threading.Thread):
    def __init__(self,t,*a):
        self._t=t
        self._a=a
        threading.Thread.__init__(self)
    def run(self):
        self._t(*self._a)

class FlowSensorsApp():    

    def __init__(self, PortName, Plot2DPos=False):
        # Path for saving current reading for data exchanges with sofa
        self.PressureDataFilePath = ScenePath + 'PressureData.txt'
        self.OldValues = [0,0,0,0,0]
        self.SerialObj = serial.Serial("/dev/"+ PortName,115200)
        
        self.vis = visdom.Visdom()

        # running stuff
        self.PauseRunning = False
        self.StopRunning = False        
        self.AppStartTime = time.time()
        self.CurrentTimeStamp = self.AppStartTime
        self.starttime = self.AppStartTime
        self.DeltaT = 0
        
        self.ExceptionMaxCount = 10
        self.ExceptionCounter = 0  
        
        self.Buff1 = SignalBufferTimeFlow.SignalBufferTime(self, 'Signal 1')
        self.Buff2 = SignalBufferTimeFlow.SignalBufferTime(self, 'Signal 2')
        self.Buff3 = SignalBufferTimeFlow.SignalBufferTime(self, 'Signal 3')
        self.Buff4 = SignalBufferTimeFlow.SignalBufferTime(self, 'Signal 4')
        self.Buff4 = SignalBufferTimeFlow.SignalBufferTime(self, 'Signal 5')
        
        self.Buff1LineName = "Flow"
        self.VisdomWin = self.vis.line(Y=self.Buff1.Data, X=self.Buff1.TimeStamps, env="main", win="Flow", name=self.Buff1LineName)
        
        
        self.ReleaseDetected = False  
        
        self.ReleaseQuarantineTime = 0.1 #s
        self.ReleaseTimestamp = 0
        self.FlowReleaseThreshold = -0.000170 #L/s        
        self.ReleaseDetected = True
        self.ReleaseTimestamp = time.time()
        

        
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
                    self.CurrentValues = [float(Val1), float(Val2), float(Val3), float(Val4),float(Val5)] 
                    print("CurrentValues: ", self.CurrentValues)                             
                    self.OldValues = self.CurrentValues            
                except Exception as woot:
                    print(woot.args)
                    print('Warning: could not convert the following string: ' + StringObj)
                    self.CurrentValues = self.OldValues          
                
                self.CurrentTimeStamp = time.time() - self.AppStartTime
                
                self.Buff1.updateBuffer(self.CurrentValues[0], self.CurrentTimeStamp) 
                self.Buff2.updateBuffer(self.CurrentValues[1], self.CurrentTimeStamp)               
                self.Buff3.updateBuffer(self.CurrentValues[2], self.CurrentTimeStamp)            
                self.Buff4.updateBuffer(self.CurrentValues[3], self.CurrentTimeStamp)            
                self.Buff4.updateBuffer(self.CurrentValues[4], self.CurrentTimeStamp)                      
            
                np.savetxt(self.PressureDataFilePath, self.CurrentValues)     

                PlotSkip = 1                                  
                
                self.detectRelease()                    
                    
                self.vis.line(Y=self.Buff1.PushedOutVol[::PlotSkip], X=self.Buff1.TimeStamps[::PlotSkip], update='replace', env="main", win=self.VisdomWin, name=self.Buff1LineName)                
                
                self.endtime = time.time()
                self.DeltaT = self.endtime - self.starttime
                self.starttime = time.time()
                
            except Exception as inst:
                print(inst.args)
                print("ouch!")
                pass
            
    def reinitOffsets(self):
        self.Buff1.findOffset()
        self.Buff2.findOffset()
        self.Buff3.findOffset()
        self.Buff4.findOffset()
        self.Buff1.PushedOutVol[-1] = 0
        self.Buff2.PushedOutVol[-1] = 0
        self.Buff3.PushedOutVol[-1] = 0
        self.Buff4.PushedOutVol[-1] = 0
        
        print(self.Buff1.Name + ', Mean: ' + str(self.Buff1.Offset) + ', Std: ' + str(self.Buff1.Std))
        print(self.Buff2.Name + ', Mean: ' + str(self.Buff2.Offset) + ', Std: ' + str(self.Buff2.Std))            
        print(self.Buff3.Name + ', Mean: ' + str(self.Buff3.Offset) + ', Std: ' + str(self.Buff3.Std))            
        print(self.Buff4.Name + ', Mean: ' + str(self.Buff4.Offset) + ', Std: ' + str(self.Buff4.Std))      
    
    def detectRelease(self):
            
            if self.ReleaseDetected == True:
                CurrentTime = time.time()
                if CurrentTime-self.ReleaseTimestamp > self.ReleaseQuarantineTime:
                    self.ReleaseDetected = False                
                    print('Quarantine over!')
                
            else:        
                Flows = np.array([self.Buff1.DataIn_LPerSec[-1],
                                  self.Buff2.DataIn_LPerSec[-1],
                                  self.Buff3.DataIn_LPerSec[-1],
                                  self.Buff4.DataIn_LPerSec[-1]])
                    
                Integrals = np.array([self.Buff1.PushedOutVol[-1],
                                      self.Buff2.PushedOutVol[-1],
                                      self.Buff3.PushedOutVol[-1],
                                      self.Buff4.PushedOutVol[-1]])
                    
                IdxIntegralMax = np.argmax(Integrals)
                IdxFlowMin = np.argmin(Flows)           
                
                FlowCriterium = (IdxFlowMin == IdxIntegralMax and Flows[IdxFlowMin] < self.FlowReleaseThreshold)
                VolumeCriterium = any(Integrals<0)
                if FlowCriterium or VolumeCriterium:
                    self.ReleaseDetected = True
                    self.ReleaseTimestamp = time.time()
                    print('Release Detected')
                    self.ContactDetected = False

MyPressureSensors = FlowSensorsApp("ttyACM0", False)