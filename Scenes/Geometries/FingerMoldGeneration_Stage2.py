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



def createMoldLid():
    
    #-----------------
    # Create mold lid
    #-----------------
    MoldLidTopDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2,-Const.MoldWallThickness,Const.MoldWallThickness, Const.ThicknessMold, -Const.MoldWallThickness, -Const.LengthMold))
    MoldLidInteriorDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2+Const.MoldWallThickness+Const.MoldCoverTolerance,0,Const.MoldCoverTolerance, Const.ThicknessMold-2*Const.MoldWallThickness-2*Const.MoldCoverTolerance, -Const.MoldWallThickness, -Const.LengthMold+2*Const.MoldWallThickness+2*Const.MoldCoverTolerance))
    FuseOut = gmsh.model.occ.fuse([MoldLidTopDimTag],[MoldLidInteriorDimTag)
    LidDimTag = FuseOut[0][0]
    
    HoleDimTag = (3,gmsh.model.occ.addBox(Const.OuterRadius*2))
    
    gmsh.model.occ.synchronize()
    gmsh.fltk.run()    
    #-----------------
    # Create cavity cork
    #-----------------
    
#LidPG = gmsh.model.addPhysicalGroup(3,[LidDimTag])
    
createMoldLid()