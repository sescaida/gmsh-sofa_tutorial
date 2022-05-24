import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np


import os

GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

import Geometries.Constants as Const

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
        self.CableConstraint1 = self.ModelNode.cables.cable1.CableConstraint
        self.CableConstraint2 = self.ModelNode.cables.cable2.CableConstraint
        self.ReferenceMO = self.RootNode.ReferenceMONode.ReferenceMO
        self.StartPosition = np.array(self.ReferenceMO.position.value[0])
        self.DistanceFromBase = np.abs(self.StartPosition[2])
        self.CurrentAngle = 0
        
        # Cavities
        self.SurfacePressureConstraint1 = self.ModelNode.Cavity01.SurfacePressureConstraint        
        self.SurfacePressureConstraint2 = self.ModelNode.Cavity02.SurfacePressureConstraint

        
        print('Finished Init')
        
    def onAnimateBeginEvent(self, eventType):
        pass
  
        
    def onKeypressedEvent(self, c):
        key = c['key']        
            
        ##########################################
        # Cable                                  #
        ##########################################                
        
        CurrentCableLength1 = np.array(self.CableConstraint1.value.value[0])
        print("CurrentCableLength1", CurrentCableLength1)
        
        CurrentCableLength2 = np.array(self.CableConstraint2.value.value[0])
        print("CurrentCableLength2", CurrentCableLength2)
        Increment = 1
        
        if (key == "0"):
            InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
            Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
            GrowthPercent = np.abs(Cavity01VolumeGrowth)/InitialCavityVolume * 100
            GrowthPerDisplacement = GrowthPercent/CurrentCableLength1
            print("GrowthPercent: ", GrowthPercent)
            print("GrowthPerDisplacement: ", GrowthPerDisplacement)
        
        if (key == "6"):
            pass
            CurrentCableLength1 = CurrentCableLength1 + Increment
            self.CableConstraint1.value = [CurrentCableLength1.tolist()]
            #self.SerialObj.writelines(CurrentCableLength1)

        if (key == "4"):
            pass
            CurrentCableLength1 = CurrentCableLength1 - Increment
            self.CableConstraint1.value = [CurrentCableLength1.tolist()]            
            
            
        if (key == "9"):
            pass
            CurrentCableLength2 = CurrentCableLength2 + Increment
            self.CableConstraint2.value = [CurrentCableLength2.tolist()]
            #self.SerialObj.writelines(CurrentCableLength1)

        if (key == "7"):
            pass
            CurrentCableLength2 = CurrentCableLength2 - Increment
            self.CableConstraint2.value = [CurrentCableLength2.tolist()]            
            
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

                VolumetricMeshPath = GeneratedMeshesPath + 'Finger_Volumetric.vtk'
                                      
                SurfaceMeshPath = GeneratedMeshesPath + 'Finger_Surface.stl'
                   
                
                model = rootNode.addChild('model')
                model.addObject('EulerImplicit', name='odesolver', rayleighStiffness=0.1)
                #model.addObject('PCGLinearSolver', name='linearSolver',iterations='25', tolerance='1.0e-9', preconditioners="precond")
                model.addObject('SparseLDLSolver', name='precond')
                #model.addObject('EigenSimplicialLDLT', name='precond', template="CompressedRowSparseMatrixMat3x3d")

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
                
                FollowingMONode = model.addChild('FollowingMONode')                
                FollowingMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[-Const.Thickness/2, Const.Height/2, -2.5*Const.Length], showObject=True, showObjectScale=20, showColor=[0,0,1]) # orientation is 240 deg away from scene origin
                #FollowingMONode.addObject('RestShapeSpringsForceField', name='fixed1', points=[0], external_rest_shape="@../../ReferenceMONode/ReferenceMO", stiffness=1e12)
                FollowingMONode.addObject("BarycentricMapping")

                ##########################################
                # Effector                               #
                ##########################################                
                
                for i in range(1,3):                    
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
                
                NSegments = 3
                CableHeight = 2*(Const.Height-Const.JointHeight)/3
                LengthDiagonal = CableHeight/np.cos(Const.JointSlopeAngle)
                JointStandoff = LengthDiagonal*np.sin(Const.JointSlopeAngle)
                CableDistance = Const.CableDistance
                BellowGap = Const.BellowHeight * (Const.NBellows-1)
                
                # Cable 1
                
                CablePoints = np.array([])
                for i in range(NSegments):
                    SegmentOffsetBase = (Const.Length+BellowGap)*i
                    SegmentOffsetTip  = Const.Length*(i+1)+BellowGap*i
                    CablePoints = np.append(CablePoints, [[-CableDistance/2,CableHeight+Const.JointHeight,-JointStandoff - SegmentOffsetBase]])
                    CablePoints = np.append(CablePoints, [[-CableDistance/2,CableHeight+Const.JointHeight, JointStandoff - SegmentOffsetTip]])
                
                cable1 = cables.addChild('cable1')
                cable1.addObject('MechanicalObject', position=CablePoints.tolist())
                
                cable1.addObject('CableConstraint', template='Vec3d', name='CableConstraint', indices=list(range(2*NSegments)), pullPoint=[0, CableHeight+Const.JointHeight, 0], printLog=True, value=10)                               
                cable1.addObject('BarycentricMapping')                
                
                # Cable 2
                
                CablePoints = np.array([])
                for i in range(NSegments):
                    SegmentOffsetBase = (Const.Length+BellowGap)*i
                    SegmentOffsetTip  = Const.Length*(i+1)+BellowGap*i
                    CablePoints = np.append(CablePoints, [[CableDistance/2,CableHeight+Const.JointHeight,-JointStandoff - SegmentOffsetBase]])
                    CablePoints = np.append(CablePoints, [[CableDistance/2,CableHeight+Const.JointHeight, JointStandoff - SegmentOffsetTip]])
                
                cable2 = cables.addChild('cable2')
                cable2.addObject('MechanicalObject', position=CablePoints.tolist())
                
                cable2.addObject('CableConstraint', template='Vec3d', name='CableConstraint', indices=list(range(2*NSegments)), pullPoint=[0, CableHeight+Const.JointHeight, 0], printLog=True, value=10)                               
                cable2.addObject('BarycentricMapping')                
                                                
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
