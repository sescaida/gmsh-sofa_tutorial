import Sofa.Core
import Sofa.Simulation
import SofaRuntime
import os
import sys
import numpy as np


import os

GeneratedMeshesPath = os.path.dirname(os.path.abspath(__file__))+'/Geometries/'

import Geometries.ConstantsTrunk as Const

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
        
        self.CablesSect1 = [self.Cable1_0, self.Cable1_1, self.Cable1_2]
        self.Cable2_0 =self.Cables.cable2_0
        self.Cable2_1 =self.Cables.cable2_1
        self.Cable2_2 =self.Cables.cable2_2
    
        self.CablesSect2 = [self.Cable2_0, self.Cable2_1, self.Cable2_2]
        
        self.Angle1 = 0 
        self.Force1 = 0
        self.Angle2 = 0
        self.Force2 = 0 
        
        self.AngleIncrement = np.deg2rad(10)
        self.ForceIncrement = 50

        
        print('Finished Init')
      
    def printConfiguration(self):
        print("------")
        print('Current configuration')
        print("Angle1: {}".format(self.Angle1))
        print("Angle2: {}".format(self.Angle2))
        print("Displacement1: {}".format(self.Force1))
        print("Displacement21: {}".format(self.Force2))
        
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
        
        if (key=='+'):
            self.Force1 += self.ForceIncrement
       
        if (key=='-'):
            self.Force1 -= self.ForceIncrement
       
#       
        if (key=='1'):
            self.Angle1 = (self.Angle1+self.AngleIncrement)%(2*np.pi)
       
        if (key=='2'):
            self.Angle1 = (self.Angle1-self.AngleIncrement)%(2*np.pi)
                   
        if (key=='4'):
            self.Force2 += self.ForceIncrement
       
        if (key=='5'):
            self.Force2 -= self.ForceIncrement
       
#       
        if (key=='7'):
            self.Angle2 = (self.Angle2+self.AngleIncrement)%(2*np.pi)
       
        if (key=='8'):
            self.Angle2 = (self.Angle2-self.AngleIncrement)%(2*np.pi)
       
            
        if (key == "0"):
            self.Angle1 = 0 
            self.Force1 = 0
            self.Angle2 = 0
            self.Force2 = 0 
        
        self.printConfiguration()   
        AngleDistance = np.deg2rad(120)

        Tension = 50        
        self.CablesSect1[0].CableConstraint.value.value = [Tension]

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
