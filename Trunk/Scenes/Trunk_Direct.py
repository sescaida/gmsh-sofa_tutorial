import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np


import os

GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

import Geometries.ConstantsTrunk as Const

#import Geometries.TrunkGeneration as Generation
#
#GenerateMeshes = False
#if GenerateMeshes:
#    Generation.createShapes()
def CircularBufferAccess(Buffer, idx):
    NElements = len(Buffer)
    return Buffer[idx%NElements]

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
        self.Cables = self.ModelNode.cables
        self.Cable1_0 =self.Cables.cable1_0
        self.Cable1_1 =self.Cables.cable1_1
        self.Cable1_2 =self.Cables.cable1_2
        #self.Cable1_3 =self.Cables.cable1_3
        self.CablesSect1 = [self.Cable1_0, self.Cable1_1, self.Cable1_2]
        self.Cable2_0 =self.Cables.cable2_0
        self.Cable2_1 =self.Cables.cable2_1
        self.Cable2_2 =self.Cables.cable2_2
        #self.Cable2_3 =self.Cables.cable2_3
        self.CablesSect2 = [self.Cable2_0, self.Cable2_1, self.Cable2_2]
        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
        pass
  
        
    def onKeypressedEvent(self, c):
        key = c['key']        
        
        Limits = np.array([0, np.deg2rad(120), np.deg2rad(240)])

            
        ##########################################
        # Cable                                  #
        ##########################################                
        
#        CurrentCableLength = np.array(self.CableConstraint.value.value[0])
#        print("CurrentCableLength", CurrentCableLength)
#        Increment = 1
#        
        if (key == "0"):
#            InputStr = input("Enter desired cable configuration! ")
            InputStr = '210,600,60,600'
            SplitStr = InputStr.split(',')
            
            Angle1 = np.deg2rad(float(SplitStr[0]))
            Displacement1 = float(SplitStr[1])
            Angle2 = np.deg2rad(float(SplitStr[2]))
            Displacement2 = float(SplitStr[3])

            AngleDistance = np.deg2rad(120)
            
                  
            for cable in self.CablesSect1:
                cable.CableConstraint.value.value = [0]
            
            Angle1BaseIdx = int(np.argmax(((Limits-Angle1)>0)))-1
            Weight1 = (Angle1-Limits[Angle1BaseIdx])/AngleDistance
            
             
            print("Idx: {}".format(Angle1BaseIdx))
            print("Weight1: {}".format(Weight1))
            
            print("Section 1: {}".format(self.CablesSect1[Angle1BaseIdx].CableConstraint.value.value))
            
            Disp1_0 = (1-Weight1)*Displacement1
            Disp1_1 = Weight1*Displacement1
            
            print("Disp1_0: {}".format(Disp1_0))
            print("Disp1_1: {}".format(Disp1_1))
            
            self.CablesSect1[Angle1BaseIdx].CableConstraint.value.value = [Disp1_0]
            CircularBufferAccess(self.CablesSect1, Angle1BaseIdx+1).CableConstraint.value.value = [Disp1_1]     
#            for cable in self.CablesSect1:
#                cable.CableConstraint.value.value = [0]
##                
#            self.CablesSect1[Angle1BaseIdx].CableConstraint.value.value = [(1-Weight)*Displacement1]
#            CircularBufferAccess(self.CablesSect1, Angle1BaseIdx+1).CableConstraint.value.value = [Weight*Displacement1]    
#            
#            
            
            for cable in self.CablesSect2:
                cable.CableConstraint.value.value = [0]
            
            Angle2BaseIdx = int(np.argmax(((Limits-Angle2)>0)))-1
            Weight = (Angle2-Limits[Angle2BaseIdx])/AngleDistance
            
             
            print("Idx: {}".format(Angle2BaseIdx))
            print("Weight: {}".format(Weight))
            
            print("Section 2: {}".format(self.CablesSect2[Angle2BaseIdx].CableConstraint.value.value))
            
            Disp2_0 = (1-Weight)*Displacement2
            Disp2_1 = Weight*Displacement2
            
            print("Disp2_0: {}".format(Disp2_0))
            print("Disp2_1: {}".format(Disp2_1))
            
            self.CablesSect2[Angle2BaseIdx].CableConstraint.value.value = [Disp2_0]
            CircularBufferAccess(self.CablesSect2, Angle2BaseIdx+1).CableConstraint.value.value = [Disp2_1]   
            
        
#            InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
#            Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
#            GrowthPercent = np.abs(Cavity01VolumeGrowth)/InitialCavityVolume * 100
#            GrowthPerDisplacement = GrowthPercent/CurrentCableLength
#            print("GrowthPercent: ", GrowthPercent)
#            print("GrowthPerDisplacement: ", GrowthPerDisplacement)
#        
#        if (key == "6"):
#            pass
#            CurrentCableLength = CurrentCableLength + Increment
#            self.CableConstraint.value = [CurrentCableLength.tolist()]
#            #self.SerialObj.writelines(CurrentCableLength)
#
#        if (key == "4"):
#            pass
#            CurrentCableLength = CurrentCableLength - Increment
#            self.CableConstraint.value = [CurrentCableLength.tolist()]            
#            
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
                rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots')
                rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

                rootNode.findData('gravity').value = [0, 0, -9810] #
                rootNode.findData('dt').value = 0.02

                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

                rootNode.addObject('BackgroundSetting', color='1 1 1')
                
                rootNode.addObject('LightManager')
                rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
                rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50")                               

                VolumetricMeshPath = GeneratedMeshesPath + 'Trunk_Volumetric.vtk'
                                      
                SurfaceMeshPath = GeneratedMeshesPath + 'Trunk_Surface.stl'
                   
                
                model = rootNode.addChild('model')
                model.addObject('EulerImplicit', name='odesolver')
                #model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
                model.addObject('SparseLDLSolver', name='precond')

                model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
                model.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                model.addObject('TetrahedronSetGeometryAlgorithms')
                model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
                model.addObject('UniformMass', totalMass='0.1')
                model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=Const.PoissonRation,  youngModulus=Const.YoungsModulus)

                BoxMargin = 3
                BoxCoords = [-(Const.Height+2*BoxMargin), -(Const.Height+2*BoxMargin), BoxMargin, Const.Height+2*BoxMargin,Const.Height+2*BoxMargin, -BoxMargin]
                model.addObject('BoxROI', name='boxROI', box=BoxCoords, drawBoxes=True)
                model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10)               
                

                model.addObject('LinearSolverConstraintCorrection', name='GCS', solverName='precond')
                
#                FollowingMONode = model.addChild('FollowingMONode')                
#                FollowingMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=20, showColor=[0,0,1]) # orientation is 240 deg away from scene origin
#                #FollowingMONode.addObject('RestShapeSpringsForceField', name='fixed1', points=[0], external_rest_shape="@../../ReferenceMONode/ReferenceMO", stiffness=1e12)
#                FollowingMONode.addObject("BarycentricMapping")

#                ##########################################
#                # Effector                               #
#                ##########################################                
#                
#                for i in range(1,5):                    
#                    CavitySurfaceMeshPath = GeneratedMeshesPath+'Cavity0' + str(i) + '.stl'                           
#                    CurrentCavity = model.addChild('Cavity0'+str(i))
#                    CurrentCavity.addObject('MeshSTLLoader', name='MeshLoader', filename=CavitySurfaceMeshPath)
#                    CurrentCavity.addObject('Mesh', name='topology', src='@MeshLoader')
#                    CurrentCavity.addObject('MechanicalObject', src="@topology")
#                    CurrentCavity.addObject('SurfacePressureConstraint', template='Vec3d', triangles='@topology.triangles')
#                    CurrentCavity.addObject('BarycentricMapping', name="Mapping", mapForces="false", mapMasses="false")

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
                
                
                CableHeight = (Const.Height-Const.JointHeight)/2
                LengthDiagonal = CableHeight/np.cos(Const.JointSlopeAngle)
                JointStandoff = LengthDiagonal*np.sin(Const.JointSlopeAngle)
                AngleCables = np.deg2rad(120)
                CableRadius = CableHeight+Const.JointHeight
                
                for k in range(1,Const.NSections+1):
                    for i in range(0,3):
                        cable = cables.addChild('cable'+str(k)+'_'+str(i))
                    
                        CablePoints = np.empty((0,3))
                        NSegments = k * Const.NSegmentsPerSection
                        AngleOffset = k*np.deg2rad(0)
                        for j in range(NSegments):
                            SegmentOffsetBase = Const.Length*j
                            SegmentOffsetTip  = Const.Length*(j+1)
                            CablePoints = np.append(CablePoints, [[np.cos(AngleCables*i+AngleOffset)*CableRadius, np.sin(AngleCables*i+AngleOffset)*CableRadius,-JointStandoff - SegmentOffsetBase]],axis=0)
                            CablePoints = np.append(CablePoints, [[np.cos(AngleCables*i+AngleOffset)*CableRadius, np.sin(AngleCables*i+AngleOffset)*CableRadius, JointStandoff - SegmentOffsetTip]],axis=0)
                        print("CablePoints: {}".format(CablePoints))
                        cable.addObject('MechanicalObject', position=CablePoints.tolist())
                        cable.addObject('CableConstraint', template='Vec3d', name='CableConstraint', indices=list(range(0,CablePoints.shape[0])), pullPoint=[np.cos(AngleCables*i+AngleOffset)*CableRadius, np.sin(AngleCables*i+AngleOffset)*CableRadius,10], printLog=True, value=0, valueType='force')                               
                        cable.addObject('BarycentricMapping')                
                                                
                ##########################################
                # Moving Point                           #
                ##########################################
                ReferenceMONode = rootNode.addChild('ReferenceMONode')
                
                ReferenceMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=10) # orientation is 240 deg away from scene origin

                ##########################################
                # Controller                             #
                ##########################################
                rootNode.addObject(Controller(name="ActuationController", RootNode=rootNode))
                
                return rootNode
