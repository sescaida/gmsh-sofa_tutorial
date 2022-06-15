import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np
import serial


import os

GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

import Geometries.Constants as Const

class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']
#        self.SerialObj = kwargs['SerialObj']
        print(kwargs['RootNode'])
        
        self.Counter = 0
        self.IterationCounter = 0
        self.DistributionStride = 5
        self.begun = False
        
        self.ModelNode = self.RootNode.model        
        self.CableConstraint = self.ModelNode.cables.cable1.CableConstraint
        #self.ReferenceMO = self.RootNode.ReferenceMONode.ReferenceMO
        #self.StartPosition = np.array(self.ReferenceMO.position.value[0])
        #self.DistanceFromBase = np.abs(self.StartPosition[2])
        self.CurrentAngle = 0
        
        # Cavities
        self.SurfacePressureConstraint1 = self.ModelNode.Cavity01.SurfacePressureConstraint        
        self.SurfacePressureConstraint2 = self.ModelNode.Cavity02.SurfacePressureConstraint
        self.SurfacePressureConstraint3 = self.ModelNode.Cavity03.SurfacePressureConstraint
        self.SurfacePressureConstraint4 = self.ModelNode.Cavity04.SurfacePressureConstraint        
        
        
        self.FileIMUData = "IMUData.txt"
        self.IMUData = None
        self.FileMotorCommands = "MotorCommands.txt"
        np.savetxt(self.FileMotorCommands, [0,0,0],fmt='%i')
        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
#        print("packetIn: {}".format(self.SerialObj.packetIn.value))
        try:
            self.IMUData = np.loadtxt(self.FileIMUData)
        except:
            "Error while reading IMU data"
        
        print("IMUData: {}".format(self.IMUData))        
        pass
  
        
    def onKeypressedEvent(self, c):
        key = c['key']        
            
        ##########################################
        # Cable                                  #
        ##########################################                
        
        CurrentCableLength = np.array(self.CableConstraint.value.value[0])
        print("CurrentCableLength", CurrentCableLength)
        Increment = 1
        
        if (key == "0"):
            InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
            Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
            GrowthPercent = np.abs(Cavity01VolumeGrowth)/InitialCavityVolume * 100
            GrowthPerDisplacement = GrowthPercent/CurrentCableLength
            print("GrowthPercent: ", GrowthPercent)
            print("GrowthPerDisplacement: ", GrowthPerDisplacement)
        
        if (key == "6"):
            pass
            CurrentCableLength = CurrentCableLength + Increment
            self.CableConstraint.value = [CurrentCableLength.tolist()]
            #self.SerialObj.writelines(CurrentCableLength)

        if (key == "4"):
            pass
            CurrentCableLength = CurrentCableLength - Increment
            self.CableConstraint.value = [CurrentCableLength.tolist()]            


        CableLengthInt = 5*int(self.CableConstraint.value[0])            
        DesiredMotorValues = np.array([CableLengthInt, CableLengthInt, CableLengthInt],dtype=int)
        np.savetxt(self.FileMotorCommands, DesiredMotorValues,fmt='%i')
#        print("sending commands to servos ...")        
#        String = str(DesiredMotorValues[0]) + ' ' + str(DesiredMotorValues[1]) + ' ' + str(DesiredMotorValues[2]) + '\n'
#        ByteString = String.encode('ASCII')
#        print("Sending to the motors: {}".format(ByteString))
#        self.SerialObj.write(ByteString)        
#        print("sent commands to servos ...")
#        ##########################################
#        # ReferenceMO                            #
#        ##########################################                
#        
#        CurrentPosition = np.array(self.ReferenceMO.position.value[0])
#        
#        self.DistanceFromBase
#        
#        print("CurrentPosition", CurrentPosition)
#        Increment = np.deg2rad(5)
#        
#        if (key == "2"):
#            pass
#            self.CurrentAngle +=  Increment
#            # Axes are inverted with respect to standard references
#            X = self.DistanceFromBase * np.sin(self.CurrentAngle)
#            Z = -self.DistanceFromBase * np.cos(self.CurrentAngle)
#            NewPosition = np.array([X+self.StartPosition[0],self.StartPosition[1],Z]) 
#            self.ReferenceMO.position.value = [NewPosition.tolist()]            
#        
#        if (key == "8"):
#            pass
#            self.CurrentAngle -=  Increment
#            # Axes are inverted with respect to standard references
#            X = self.DistanceFromBase * np.sin(self.CurrentAngle)
#            Z = -self.DistanceFromBase * np.cos(self.CurrentAngle)
#            NewPosition = np.array([X+self.StartPosition[0],self.StartPosition[1],Z]) 
#            self.ReferenceMO.position.value = [NewPosition.tolist()]            

        
        

def createScene(rootNode):

                print('asd')
                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots EigenLinearSolvers')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -9810] #
                rootNode.findData('dt').value = 0.02

                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                rootNode.addObject('BackgroundSetting', color='1 1 1')
                
                rootNode.addObject('LightManager')
                rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
                rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50")                               

                VolumetricMeshPath = GeneratedMeshesPath + 'Finger_Volumetric.vtk'
                                      
                SurfaceMeshPath = GeneratedMeshesPath + 'Finger_Surface.stl'
                   
                
                model = rootNode.addChild('model')
                model.addObject('EulerImplicit', name='odesolver')
                #model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
                #model.addObject('SparseLDLSolver', name='precond')
                model.addObject('EigenSimplicialLDLT', name='precond', template="CompressedRowSparseMatrixMat3x3d")

                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                #model.addObject('TetrahedronSetTopologyContainer', name='container', triangles='@loader.triangles', tetrahedra='@loader.tetrahedra')
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
                    CurrentCavity.addObject('SurfacePressureConstraint', template='Vec3d', triangles='@topology.triangles')
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
                
                cables = model.addChild('cables')
                cable1 = cables.addChild('cable1')
                
                NSegments = 3
                CableHeight = 2*(Const.Height-Const.JointHeight)/3
                LengthDiagonal = CableHeight/np.cos(Const.JointSlopeAngle)
                JointStandoff = LengthDiagonal*np.sin(Const.JointSlopeAngle)
                
                CablePoints = np.array([])
                for i in range(NSegments):
                    SegmentOffsetBase = Const.Length*i
                    SegmentOffsetTip  = Const.Length*(i+1)
                    CablePoints = np.append(CablePoints, [[0,CableHeight+Const.JointHeight,-JointStandoff - SegmentOffsetBase]])
                    CablePoints = np.append(CablePoints, [[0,CableHeight+Const.JointHeight, JointStandoff - SegmentOffsetTip]])
                
                cable1.addObject('MechanicalObject', position=CablePoints.tolist())
                
                cable1.addObject('CableConstraint', template='Vec3d', name='CableConstraint', indices=list(range(2*NSegments)), pullPoint=[0, CableHeight+Const.JointHeight, 0], printLog=True, value=0)                               
                cable1.addObject('BarycentricMapping')                
                                                
                ##########################################
                # Moving Point                           #
                ##########################################
#                ReferenceMONode = rootNode.addChild('ReferenceMONode')
#                
#                ReferenceMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=10) # orientation is 240 deg away from scene origin

                ##########################################
                # Controller                             #
                ##########################################
#                serialport='/dev/ttyACM0'
#                SerialObj = rootNode.addObject("SerialPortBridgeGeneric", port=serialport, baudRate=115200, size=3, listening=True, receive=True, header=255)
#                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode, SerialObj=SerialObj))
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode))
                
                
                
                return rootNode
