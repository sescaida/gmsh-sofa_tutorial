import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np


import os

GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

import Geometries.Constants as Const

def processContactMask(Mask):
    NSensors = 6
    Contacts = np.zeros(NSensors,bool)
    
    for i in range(NSensors):
        if(2**i & Mask):
            Contacts[i] = True
    print("Contacts: " , Contacts)
    return Contacts[1:]

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
        print(kwargs['RootNode'])
        
        self.Counter = 0
        self.IterationCounter = 0
        self.DistributionStride = 5
        self.begun = False
        
        self.ModelNode = self.RootNode.model        
        self.CableActuator = self.ModelNode.CableNode.CableActuator
        self.FPA1 = self.ModelNode.PushingActuationNode.FPA1
        self.FPA2 = self.ModelNode.PushingActuationNode.FPA2
        #self.ReferenceMO = self.RootNode.ReferenceMONode.ReferenceMO
        #self.StartPosition = np.array(self.ReferenceMO.position.value[0])
#        self.DistanceFromBase = np.abs(self.StartPosition[2])
#        self.CurrentAngle = 0
#        
        # Cavities
        self.VolumeEffector1 = self.ModelNode.Cavity01.VolumeEffector        
        self.VolumeEffector2 = self.ModelNode.Cavity02.VolumeEffector
        self.VolumeEffector3 = self.ModelNode.Cavity03.VolumeEffector
        self.VolumeEffector4 = self.ModelNode.Cavity04.VolumeEffector

        #self.ForceSurfaceActuator = self.ModelNode.actuation.ForceSurfaceActuator     
        self.MeasuredVolumeChange = 0
        self.VolumeIncrementSensor = 3
        self.DesiredVolumeIncrement = 300
        self.SideInflationSign = 1
        self.LivePressureDataPath = "PressureData.txt"
        self.LiveMuCaDataPath = "MuCaData.txt"
        self.Data = np.array([])
        self.LastData = self.Data
        NSensors = 5
        self.Contacts = np.zeros(NSensors,int)
        self.ContactIdxs = np.array([0,1,2,3,4,5])
        self.ContactDirections = np.array([[1,0,0],[0,-1,0], [-1,0,0],[1,0,0],[0,-1,0],[-1,0,0]])
        
        
        print('Finished Init')
            
        
#    def onEvent(self, event): 
#        print(event)
#

    def onAnimateBeginEvent(self, eventType):       
                
        
        try:
            self.Data = np.loadtxt(self.LivePressureDataPath)
            self.RealPressures = self.Data[:4] # in kPa            
            self.MuCaData = np.loadtxt(self.LiveMuCaDataPath)
            
        except Exception as e:
            print("Warning: couldn't read pressures or contact mask from file")
#            self.Data = self.LastData
#            self.RealPressures = self.Data[:6] # in kPa   
            return
        
        
        print('Pressures: ' +  str(self.RealPressures))
        if self.Data.size == 0:
                return
               
        
        self.InitialVolumeCavity1 = self.VolumeEffector1.initialCavityVolume.value
        self.InitialVolumeCavity2 = self.VolumeEffector2.initialCavityVolume.value
        self.InitialVolumeCavity3 = self.VolumeEffector3.initialCavityVolume.value
        self.InitialVolumeCavity4 = self.VolumeEffector4.initialCavityVolume.value
        
        FactorU = 10
        Factor = 3
        #Factor = 17
        AtmPressure = 101 #kPa
        # P*V is constant, Ci are theses constants as found by the initial volume. A factor is introduced to account for tubing and volume inside the pressure sensor        
        
        
        C1 = self.InitialVolumeCavity1 * AtmPressure 
        C2 = self.InitialVolumeCavity2 * AtmPressure 
        C3 = self.InitialVolumeCavity3 * AtmPressure 
        C4 = self.InitialVolumeCavity4 * AtmPressure     
        
        VolumeChangeCavity1 = (self.InitialVolumeCavity1 - C1/(AtmPressure + self.RealPressures[0])) * Factor
        VolumeChangeCavity2 = (self.InitialVolumeCavity2 - C2/(AtmPressure + self.RealPressures[1])) * Factor
        VolumeChangeCavity3 = (self.InitialVolumeCavity3 - C3/(AtmPressure + self.RealPressures[2])) * Factor
        VolumeChangeCavity4 = (self.InitialVolumeCavity4 - C4/(AtmPressure + self.RealPressures[3])) * Factor
        
        self.setDesiredVolumeEffectors([VolumeChangeCavity1, VolumeChangeCavity2, VolumeChangeCavity3, VolumeChangeCavity4])
        
        self.LastData = self.Data
        
        
        #-----------
        # process contacts
        #-----------
        
        print("MuCa: {}".format(self.MuCaData))
        if len(self.MuCaData) == 0:
            print("empty!")
            return
        
        Idxs = []
        MeanSect1 = np.mean(self.MuCaData[0,1:])
        print("Mean Sect1: {}".format(MeanSect1))
        DetectionThreshold1 = 100
        DetectionThreshold2 = 60
        DetectionThreshold3 = 100
        if ((self.MuCaData[0,1]-MeanSect1)> DetectionThreshold1):
            Idxs = Idxs +  [0]
        elif ((self.MuCaData[0,2]-MeanSect1)> DetectionThreshold2):
            Idxs = Idxs +  [1]
        elif ((self.MuCaData[0,3]-MeanSect1)> DetectionThreshold3):
            Idxs = Idxs +  [2]
            
        MeanSect2 = np.mean(self.MuCaData[1,1:])
        print("Mean Sect2: {}".format(MeanSect2))
        DetectionThreshold3 = 100
        DetectionThreshold4 = 70
        DetectionThreshold5 = 100
        if ((self.MuCaData[1,1]-MeanSect1)> DetectionThreshold3):
            Idxs = Idxs +  [3]
#        elif ((self.MuCaData[1,2]-MeanSect1)> DetectionThreshold4):
#            Idxs = Idxs +  [4]
#        elif ((self.MuCaData[1,3]-MeanSect1)> DetectionThreshold5):
#            Idxs = Idxs +  [5]
#            
#            
        
#        DetectionThreshold1 = 400
#        DetectionThreshold2 = 200
#        DetectionThreshold3 = 400
#        DetectionThreshold4 = 300
#        Idxs = []
#        
#        if (self.MuCaData[1,3] - self.MuCaData[1,2]) > DetectionThreshold1:
#            Idxs = Idxs +  [0]
#        elif (self.MuCaData[1,2] - self.MuCaData[1,3]) > DetectionThreshold2:
#            Idxs = Idxs +  [2]
#        
#        if (self.MuCaData[0,1] - self.MuCaData[0,0]) > DetectionThreshold3:
#            Idxs = Idxs +  [3]
#        elif (self.MuCaData[0,0] - self.MuCaData[0,1]) > DetectionThreshold4:
#            Idxs = Idxs +  [5]
#            
            
#        Idxs = self.ContactIdxs[self.Contacts]
        print("Idxs: ", Idxs)
        if len(Idxs)>0:
            self.FPA1.direction=self.ContactDirections[Idxs[0]].tolist()
            self.FPA1.indices=[Idxs[0]]
            if len(Idxs)>1:
                self.FPA2.direction=self.ContactDirections[Idxs[1]].tolist()
                self.FPA2.indices=[Idxs[1]]
            else:
                self.FPA2.indices=[]            
        else:
            self.FPA1.indices=[]
            
        
        
        
        
    
    def getReachedVolumes(self):
    
        Vol1 = self.VolumeEffector1.initialCavityVolume.value - self.VolumeEffector1.cavityVolume.value 
        Vol2 = self.VolumeEffector2.initialCavityVolume.value - self.VolumeEffector2.cavityVolume.value 
        ReachedVolumes = np.array([Vol1,Vol2])
        return ReachedVolumes

    def setDesiredVolumeEffectors(self, Volumes): 

        #self.ForceBM.reinit()       
        
        #Volumes[0] = Volumes[0]*0.794718462  #using a correction factor here
        print('Desired volume changes (V1,V2,V3,V4): ' + str(Volumes))
        self.VolumeEffector1.getData('desiredVolume').value = self.VolumeEffector1.initialCavityVolume.value - Volumes[0]
        self.VolumeEffector2.getData('desiredVolume').value = self.VolumeEffector2.initialCavityVolume.value - Volumes[1]
        self.VolumeEffector3.getData('desiredVolume').value = self.VolumeEffector3.initialCavityVolume.value - Volumes[2]
        self.VolumeEffector4.getData('desiredVolume').value = self.VolumeEffector4.initialCavityVolume.value - Volumes[3]

        
    def onKeypressedEvent(self, c):
        key = c['key']        
                        

def createScene(rootNode):
    
                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')# EigenLinearSolvers')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -9810] #
                rootNode.findData('dt').value = 0.02

                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject("QPInverseProblemSolver", printLog=1, epsilon=0.1, maxIterations=1000,tolerance=1e-5)

                rootNode.addObject('BackgroundSetting', color='1 1 1')
                
                rootNode.addObject('LightManager')
                rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
                rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50")                               

                VolumetricMeshPath = GeneratedMeshesPath + 'Finger_Volumetric.vtk'
                                      
                SurfaceMeshPath = GeneratedMeshesPath + 'Finger_Surface.stl'
                   
                
                model = rootNode.addChild('model')
#                model.addObject('EulerImplicit', name='odesolver',rayleighStiffness=0.1)
                model.addObject('EulerImplicit', name='odesolver',rayleighStiffness=0.1)
                #model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
                model.addObject('SparseLDLSolver', name='precond')
                #model.addObject('EigenSimplicialLDLT', name='precond', template="CompressedRowSparseMatrixMat3x3d")

                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                model.addObject('TetrahedronSetGeometryAlgorithms')
                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
                model.addObject('UniformMass', totalMass='0.1')
                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Const.PoissonRation,  youngModulus=Const.YoungsModulus)

                BoxMargin = 3
                BoxCoords = [-(Const.Thickness/2+BoxMargin), -BoxMargin, BoxMargin, Const.Thickness/2+BoxMargin,Const.Height+2*BoxMargin, -BoxMargin]
                model.addObject('BoxROI', name='boxROI', box=BoxCoords, drawBoxes=True)
                model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10)               
                

                model.addObject('LinearSolverConstraintCorrection', name='GCS', solverName='precond')
                
#                FollowingMONode = model.addChild('FollowingMONode')                
#                FollowingMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=20, showColor=[0,0,1]) # orientation is 240 deg away from scene origin
#                #FollowingMONode.addObject('RestShapeSpringsForceField', name='fixed1', points=[0], external_rest_shape="@../../ReferenceMONode/ReferenceMO", stiffness=1e12)
#                FollowingMONode.addObject("BarycentricMapping")

                ##########################################
                # Effector                               #
                ##########################################                
                
                for i in range(1,5):                    
                    CavitySurfaceMeshPath = GeneratedMeshesPath+'Cavity0' + str(i) + '.stl'                           
                    CurrentCavity = model.addChild('Cavity0'+str(i))
                    CurrentCavity.addObject('MeshSTLLoader', name='MeshLoader', filename=CavitySurfaceMeshPath)
                    CurrentCavity.addObject('Mesh', name='topology', src='@MeshLoader')
                    CurrentCavity.addObject('MechanicalObject', src="@topology")
                    CurrentCavity.addObject('VolumeEffector', template='Vec3d', triangles='@topology.triangles')
                    CurrentCavity.addObject('BarycentricMapping', name="Mapping", mapForces="false", mapMasses="false")

		        ##########################################
                # Visualization                          #
                ##########################################

                modelVisu = model.addChild('visu')
                modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
                modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
                modelVisu.addObject('BarycentricMapping')

                ##########################################
                # Cable Actuation                        #
                ##########################################
                
                CableNode = model.addChild('CableNode')
                
                
                NSegments = 3
                CableHeight = 2*(Const.Height-Const.JointHeight)/3
                LengthDiagonal = CableHeight/np.cos(Const.JointSlopeAngle)
                JointStandoff = LengthDiagonal*np.sin(Const.JointSlopeAngle)
                JointStandoffExtra = 2
                CablePoints = np.array([])
                for i in range(NSegments):
                    SegmentOffsetBase = Const.Length*i
                    SegmentOffsetTip  = Const.Length*(i+1)
                    CablePoints = np.append(CablePoints, [[0,CableHeight+Const.JointHeight,-(JointStandoff+JointStandoffExtra) - SegmentOffsetBase]])
                    CablePoints = np.append(CablePoints, [[0,CableHeight+Const.JointHeight, JointStandoff+JointStandoffExtra - SegmentOffsetTip]])
                
                CableNode.addObject('MechanicalObject', position=CablePoints.tolist())
                
                CableNode.addObject('CableActuator', template='Vec3d', name='CableActuator', indices=list(range(2*NSegments)), pullPoint=[0, CableHeight+Const.JointHeight, 0], printLog=True)                               
                CableNode.addObject('BarycentricMapping')             
                
                ##########################################
                # Pushing Actuation                      #
                ##########################################
                
                PushingActuationNode = model.addChild('PushingActuationNode')                
                NSegments = 3                                
                PushingPoints = np.array([[Const.Thickness/2, Const.Height/2, -1.5*Const.Length],
                                          [0, Const.Height, -1.5*Const.Length],
                                          [-Const.Thickness/2, Const.Height/2, -1.5*Const.Length],
                                          [Const.Thickness/2, Const.Height/2, -2.5*Const.Length],
                                          [0, Const.Height, -2.5*Const.Length],
                                          [-Const.Thickness/2, Const.Height/2, -2.5*Const.Length],
                                          ])
                
                
                PushingActuationNode.addObject('MechanicalObject', position=PushingPoints.tolist(),showObject=True, showColor=[0,0,1],showObjectScale=5)
                
                FPA1 = PushingActuationNode.addObject('ForcePointActuator', name="FPA1", direction=[1,0,0], indices = [0], showForce=True,visuScale=0.2, template='Vec3d', printLog=True)                               
                FPA2 = PushingActuationNode.addObject('ForcePointActuator', name="FPA2", direction=[1,0,0], indices = [], showForce=True,visuScale=0.2, template='Vec3d', printLog=True)                               
                PushingActuationNode.addObject('BarycentricMapping')        
                                                
#                ##########################################
#                # Moving Point                           #
#                ##########################################
#                ReferenceMONode = rootNode.addChild('ReferenceMONode')
#                
#                ReferenceMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=10) # orientation is 240 deg away from scene origin

                ##########################################
                # Controller                             #
                ##########################################
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode))
                
                return rootNode
