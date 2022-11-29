import numpy as np
import datetime
import time

class SignalBufferTime:
    def __init__(self, Parent, Name='Signal', TimeWindowSize=2):
        self.Name = Name
        self.Parent = Parent        
        self.DataRaw = np.array([0,0]) # put in some values so that operations later don't fail ...        
        self.Data = np.array([0,0])
        self.DataIn_LPerSec = np.array([0,0])
        self.TimeStamps = np.array([0,0])
        self.Offset = 0
        self.Mean = 0
        self.Std = 0
        self.Threshold = 1
        self.PushedOutVol = np.array([0,0]) # Integral of the signal
        self.FlowAccel = np.array([0,0]) # Derivative of the signal       
        self.TimeWindowSize = TimeWindowSize #seconds
        self.NewValue = None
        
        
    def flushBuffer(self):        
        #print('Flushing Buffer ' + self.Name)
        self.DataRaw = np.array([0,0]) # put in some values so that operations later don't fail ...        
        self.Data = np.array([0,0])
        self.DataIn_LPerSec = np.array([0,0])
        self.TimeStamps = np.array([0,0])        
        self.PushedOutVol = np.array([0,0]) # Integral of the signal
        self.FlowAccel = np.array([0,0]) # Derivative of the signal               
        
    def updateBuffer(self, NewValue, TimeStamp):               
        
        self.CurrentTimeStamp = TimeStamp
        self.TimeStamps = np.append(self.TimeStamps, TimeStamp)
        self.DataRaw = np.append(self.DataRaw, NewValue)   
        self.NewValue = NewValue
        self.Mean = np.mean(self.DataRaw)
        self.Data = self.DataRaw - self.Offset
        
        # AD2V = 1.0/205.0; # 1024/5V = 205
        # V2mLPerMin = 0.05; # As seen from the datasheet, where in the range of 2 volts the flowrate is 0-100mL/m
        # LPerMin2LPerSec = 1.0/60.0;
        
        self.DataIn_LPerSec = self.Data #* AD2V*V2mLPerMin*LPerMin2LPerSec                
   
        # Calculate (pseudo) integral to have an approximate measure of total volume (not reliable in case of saturation)
        
        if not self.Parent.ReleaseDetected:           
            # #if abs(self.DataIn_LPerSec[-1]) > 0.000006: # for the future (needs more robustness)
            if abs(self.DataIn_LPerSec[-1]) > 0.00005: # for the future (needs more robustness)                
                DeltaT = self.TimeStamps[-1]-self.TimeStamps[-2]                                                    
                self.PushedOutVol = np.append(self.PushedOutVol, self.PushedOutVol[-1] + DeltaT*abs(self.DataIn_LPerSec[-1]))
            else:
                self.PushedOutVol = np.append(self.PushedOutVol, self.PushedOutVol[-1])
        else:                
            self.PushedOutVol = np.append(self.PushedOutVol, 0)
                
        TruncationIdx = self.findTimeTruncationIdx(self.TimeWindowSize, self.CurrentTimeStamp)
        self.truncateBuffer(TruncationIdx)
        
   
    def truncateBuffer(self, NElements):
        # truncate the buffer from the beginning, so that the first NElements are thrown out of the array
        # This function is necessary to be able to hold only elements within a time range in the array
        # The length of the buffer will no longer be constant!
        self.TimeStamps = self.TimeStamps[NElements:]
        self.DataRaw = self.DataRaw[NElements:]
        self.Data = self.Data[NElements:]
        self.DataIn_LPerSec = self.DataIn_LPerSec[NElements:]        
        self.FlowAccel = self.FlowAccel[NElements:]
        self.PushedOutVol = self.PushedOutVol[NElements:]
         
       
    def findOffset(self):
        self.Offset = self.Mean
        self.Std = np.std(self.DataRaw)
        
    def findTimeTruncationIdx(self, TimeWindowSize, CurrentTimeStamp):
        Idx = 0
        Found = False
        TimeThreshold = CurrentTimeStamp - TimeWindowSize
        while not Found:
            if Idx >= len(self.TimeStamps):
                Idx = 0
                print('Could not find right index to truncate --> not truncating!')
                print('Name: ' + self.Name)
                print(self.TimeStamps)                
                print(self.Data)
                print(self.NewValue)
                return
            elif self.TimeStamps[Idx] < TimeThreshold:
                Idx = Idx + 1
            else:
                Found = True                
        return Idx
    
    def saveBuffer(self, Name, Path='/home/stefan/Repos/PneumaticSensing/ForceTestBed/Data'):
        t = datetime.datetime.now()
        DateTimeStr = t.strftime('%Y-%m-%d_%H:%M:%S')
        np.savez(Path+'/' + DateTimeStr + '_' + Name, DataRaw=self.DataRaw)
