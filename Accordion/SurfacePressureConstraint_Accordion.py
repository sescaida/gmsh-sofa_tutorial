import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName='SoftRobots Sofa.GL.Component.Rendering3D Sofa.Component.LinearSolver.Direct Sofa.Component.LinearSolver.Preconditioner  Sofa.Component.LinearSolver.Iterative')
                rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

                rootNode.addObject('FreeMotionAnimationLoop')
                rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance = 0.0000001)

                		

                bunny = rootNode.addChild('bunny')
                bunny.addObject('EulerImplicitSolver', name='odesolver')
                bunny.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5, preconditioners='preconditioner', use_precond=True, update_step=1)

                bunny.addObject('MeshVTKLoader', name='loader', filename='Accordion_Volumetric.vtk')
                bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                bunny.addObject('TetrahedronSetTopologyModifier')

                bunny.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
                bunny.addObject('UniformMass', totalMass=0.5)
                bunny.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=18000)

                bunny.addObject('BoxROI', name='boxROI', box=[-15, -7, -15,  15, -3, 15], drawBoxes=True, position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
                bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)

                bunny.addObject('SparseLDLSolver', name='preconditioner')
                bunny.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')
                #bunny.addObject('UncoupledConstraintCorrection')


		
                cavity = bunny.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename='Accordion_Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=32, valueType=0)
                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)                
                
		        ##########################################
                # Visualization                          #
                ##########################################
                
                # Visualization                          
                modelVisu = bunny.addChild('visu')
                modelVisu.addObject('MeshSTLLoader', filename="Accordion_Surface.stl", name="loader")
                modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
                modelVisu.addObject('BarycentricMapping')

                



                return rootNode


