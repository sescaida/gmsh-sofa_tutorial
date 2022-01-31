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


#gmsh.initialize()
#gmsh.option.setNumber("Mesh.MeshOnlyVisible",1)
#gmsh.option.setNumber("General.Terminal", 1)

def createMoldLid_Stage2():
    
    #-----------------
    # Create mold lid
    #-----------------
    MoldLidTopDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2,-Const.MoldWallThickness,Const.MoldWallThickness, Const.ThicknessMold, -Const.MoldWallThickness, -Const.LengthMold))
    MoldLidInteriorDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2+Const.MoldWallThickness+Const.MoldCoverTolerance,0,Const.MoldCoverTolerance, Const.ThicknessMold-2*Const.MoldWallThickness-2*Const.MoldCoverTolerance, -Const.MoldWallThickness, -Const.LengthMold+2*Const.MoldWallThickness+2*Const.MoldCoverTolerance))
    FuseOut = gmsh.model.occ.fuse([MoldLidTopDimTag], [MoldLidInteriorDimTag])
    MoldLidIntermediateDimTag = FuseOut[0][0]
    LidHole1DimTag = (3,gmsh.model.occ.addBox(-Const.LidHoleThickness/2,0,Const.LidHoleLength/2,Const.LidHoleThickness,-2*Const.MoldWallThickness,-Const.LidHoleLength))
    LidHole2DimTag = gmsh.model.occ.copy([LidHole1DimTag])[0]
    gmsh.model.occ.translate([LidHole1DimTag],0,0,-1.5*Const.Length)
    gmsh.model.occ.translate([LidHole2DimTag],0,0,-2.5*Const.Length)
    
    CutOut = gmsh.model.occ.cut([MoldLidIntermediateDimTag],[LidHole1DimTag,LidHole2DimTag])
    MoldLidWithHolesDimTag = CutOut[0][0]
    gmsh.model.occ.synchronize()
    gmsh.write("MoldLid_Stage2.step")
    gmsh.fltk.run()    
    #-----------------
    # Create cavity cork
    #-----------------
    
#LidPG = gmsh.model.addPhysicalGroup(3,[LidDimTag])
    
def createCorkMold():
    
    CavityCorkSketchDimTag = (2,FingerGeneration.createCavitySketch(Const.OuterRadius, Const.NBellowSteps, Const.StepHeight, Const.TeethRadius, Const.WallThickness/2, Const.CenterThickness))    
    Tolerance = 2
    TotalBellowHeight = Const.NBellowSteps * Const.StepHeight * 2
    CorkMoldHeight = TotalBellowHeight + 2 * Tolerance
    CorkMoldThickness = (Const.OuterRadius+Tolerance) * 2    
    ExtrudeDimTags = gmsh.model.occ.extrude([CavityCorkSketchDimTag],0,Const.CavityCorkThickness,0)    
    HalfDimTag = ExtrudeDimTags[1]
        
    HalfCopyDimTag = gmsh.model.occ.copy([HalfDimTag])
    print("HalfCopyDimTag: ", HalfCopyDimTag)
    gmsh.model.occ.affineTransform(HalfCopyDimTag, [1,0,0,0, 0,1,0,0, 0,0,-1,0])
     
    FusionOut = gmsh.model.occ.fuse([HalfDimTag], HalfCopyDimTag)
    CavityCorkDimTag = FusionOut[0][0]
    CavityCork2DimTags = gmsh.model.occ.copy([CavityCorkDimTag])
    gmsh.model.occ.affineTransform(CavityCork2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    FuseOut = gmsh.model.occ.fuse([CavityCorkDimTag],CavityCork2DimTags)
    
    CompleteCorkDimTag = FuseOut[0][0]
    CavityMoldBoxDimTag = (3,gmsh.model.occ.addBox(-CorkMoldThickness/2,
                                                   -Tolerance,
                                                   -CorkMoldHeight/2,
                                                   CorkMoldThickness, 
                                                   Const.CavityCorkThickness+Tolerance,
                                                   CorkMoldHeight))
    CutOut = gmsh.model.occ.cut([CavityMoldBoxDimTag],[CompleteCorkDimTag])
    
    
def createMoldHoleLid():
    MoldHole1DimTag = (3,gmsh.model.occ.addBox(-Const.MoldHoleThickness/2,0,Const.MoldHoleLength/2,Const.MoldHoleThickness,2*Const.MoldWallThickness,-Const.MoldHoleLength))    
    MoldHoleLidDimTag = (3,gmsh.model.occ.addBox(-(Const.MoldHoleThickness/2+Const.MoldHoleLidBorderThickness),
                                                 0,
                                                 Const.MoldHoleLength/2+Const.MoldHoleLidBorderThickness,
                                                 Const.MoldHoleThickness+2*Const.MoldHoleLidBorderThickness,
                                                 2*Const.MoldWallThickness/2,
                                                 -(Const.MoldHoleLength+2*Const.MoldHoleLidBorderThickness)))
    gmsh.model.occ.translate([MoldHole1DimTag],0,Const.Height,-1.5*Const.Length)
    gmsh.model.occ.translate([MoldHoleLidDimTag],0,Const.Height+2*Const.MoldWallThickness,-1.5*Const.Length)
    FuseOut = gmsh.model.occ.fuse([MoldHole1DimTag],[MoldHoleLidDimTag])
    
    
def createMold_Stage2():
    MoldDimTag, Lid_Stage1_DimTag = FingerMoldGeneration.createFingerMold()
    MoldHole1DimTag = (3,gmsh.model.occ.addBox(-Const.MoldHoleThickness/2,0,Const.MoldHoleLength/2,Const.MoldHoleThickness,2*Const.MoldWallThickness,-Const.MoldHoleLength))    
    MoldHole2DimTag = gmsh.model.occ.copy([MoldHole1DimTag])[0]
    gmsh.model.occ.translate([MoldHole1DimTag],0,Const.Height,-1.5*Const.Length)
    gmsh.model.occ.translate([MoldHole2DimTag],0,Const.Height,-2.5*Const.Length)    
    CutOut = gmsh.model.occ.cut([MoldDimTag], [MoldHole1DimTag, MoldHole2DimTag])
    #MoldWithHolesDimTag = CutOut[0][0]
    
#createMold_Stage2()
#createMoldHoleLid()
createCorkMold()
gmsh.model.occ.synchronize()
gmsh.fltk.run()
    
#createMoldLid_Stage2()