#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 16:31:27 2022

@author: bionicrobotics
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 20 14:25:13 2022

@author: stefan
"""

import gmsh 
import numpy as np
import Constants as Const
import FingerGeneration
import FingerMoldGeneration

def createMoldStage2():
    # Fill in the code to create the Stage 2 mold! Hint: Use MoldDimTag and make holes in it. Lid_Stage1_DimTag can be ignored!
    MoldDimTag, Lid_Stage1_DimTag = FingerMoldGeneration.createFingerMold()
    
    # ....
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldStage2.stl")
    gmsh.fltk.run()    
    

def createMoldLidStage2():
    # Create a lid for the Stage 2 mold! Hint: you can get a lot of inspiration from the code in FingerMoldGeneration.createMoldLid()
    
    # ....
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldLidStage2.stl")
    gmsh.fltk.run()    
    
def createHoleLidForMoldStage2():
    # Create a lid to cover the holes in Stage 2 mold after the sensors have been overcast! 
    
    # .... 

    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("HoleLidForMoldStage2.stl")
    gmsh.fltk.run()    
        
def creakteMoldForCork():
    # Attention: This task is optional and will get you 1 pt extra in your course grade!
    # Create a mold to create the cork piece that will seal the cavities! Hint: the code is very similar to what can be found in FingerMoldGeneration.createMoldLid()
    
    # .... 

    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldForCork.stl")
    gmsh.fltk.run()    
    
    
def createFingerClamp():
    # Attention: This task is optional and will get you 1 pt extra in your course grade!
    # Create a clamp for the finger having to holes to fix it to a rig! Hint: use the geomtry of the Finger,
    
    MoldDimTag, Lid_Stage1_DimTag = FingerMoldGeneration.createFingerMold()
    
    CableHeight = 5*Const.Height/6
    CableLength = Const.LengthMold+2*Const.MoldWallThickness
    CableDimTag = (3,gmsh.model.occ.addCylinder(0,CableHeight,2*Const.MoldWallThickness,0,0,-CableLength,Const.CableRadius))
    
    # .... 

    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("FingerClamp.stl")
    gmsh.fltk.run()    
    
  

#createMoldStage2()
#createMoldLidStage2()
#createHoleLidForMoldStage2()
#creakteMoldForCork()
#createFingerClamp()
gmsh.model.occ.synchronize()
gmsh.fltk.run()
