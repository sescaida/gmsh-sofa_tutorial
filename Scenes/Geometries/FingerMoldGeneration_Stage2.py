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



gmsh.initialize()
gmsh.option.setNumber("Mesh.MeshOnlyVisible",1)
gmsh.option.setNumber("General.Terminal", 1)

ThicknessMold = 2*Const.OuterRadius + 2*Const.MoldWallThickness
LengthMold = 3*Const.Length + 2*Const.MoldWallThickness
HeightMold = Const.Height + Const.FixationWidth + Const.MoldWallThickness    

def createMoldLid():
    
    #-----------------
    # Create mold lid
    #-----------------
    MoldLidTopDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2,-Const.MoldWallThickness,Const.MoldWallThickness, ThicknessMold, -Const.MoldWallThickness, -LengthMold))
    MoldLidInteriorDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2+Const.MoldWallThickness+Const.MoldCoverTolerance,0,Const.MoldCoverTolerance, ThicknessMold-2*Const.MoldWallThickness-2*Const.MoldCoverTolerance, -Const.MoldWallThickness, -LengthMold+2*Const.MoldWallThickness+2*Const.MoldCoverTolerance))
    gmsh.model.occ.synchronize()
    gmsh.fltk.run()    
    #-----------------
    # Create cavity cork
    #-----------------
    
#LidPG = gmsh.model.addPhysicalGroup(3,[LidDimTag])


    
createMoldLid()