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
    MoldDimTag, Lid_Stage1_DimTag = FingerMoldGeneration.createFingerMold(Stage1Mod=False)
    MoldHole1DimTag = (3,gmsh.model.occ.addBox(-Const.MoldHoleThickness/2,0,Const.MoldHoleLength/2,Const.MoldHoleThickness,2*Const.MoldWallThickness,-Const.MoldHoleLength))    
    MoldHole2DimTag = gmsh.model.occ.copy([MoldHole1DimTag])[0]
    BoxFixationDimTag = (3,gmsh.model.occ.addBox(-(Const.Thickness/2+Const.FixationWidth),
                                              -Const.MoldWallThickness,
                                              2*Const.FixationWidth,
                                              Const.Thickness+2*Const.FixationWidth, 
                                              Const.Height+2*Const.FixationWidth,
                                              -2*Const.FixationWidth))
    gmsh.model.occ.translate([MoldHole1DimTag],0,Const.Height,-1.5*Const.Length)
    gmsh.model.occ.translate([MoldHole2DimTag],0,Const.Height,-2.5*Const.Length)    
    CutOut = gmsh.model.occ.cut([MoldDimTag], [MoldHole1DimTag, MoldHole2DimTag,BoxFixationDimTag])
    gmsh.model.occ.synchronize()
    gmsh.write("MoldStage2.step")
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldStage2.stl")
    gmsh.fltk.run()
    #MoldWithHolesDimTag = CutOut[0][0]


def createMoldLidStage2():
    
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
    gmsh.write("MoldLidStage2.step")
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldLidStage2.stl")
    gmsh.fltk.run()    
    #-----------------
    # Create cavity cork
    #-----------------
    
def createHoleLidForMoldStage2():
    Tolerance = 0.1
    MoldHole1DimTag = (3,gmsh.model.occ.addBox(-(Const.MoldHoleThickness/2+Tolerance),0,Const.MoldHoleLength/2+Tolerance,Const.MoldHoleThickness+2*Tolerance,2*Const.MoldWallThickness,-(Const.MoldHoleLength+2*Tolerance)))   
    MoldHoleLidDimTag = (3,gmsh.model.occ.addBox(-(Const.MoldHoleThickness/2+Const.MoldHoleLidBorderThickness),
                                                 0,
                                                 Const.MoldHoleLength/2+Const.MoldHoleLidBorderThickness,
                                                 Const.MoldHoleThickness+2*Const.MoldHoleLidBorderThickness,
                                                 2*Const.MoldWallThickness/2,
                                                 -(Const.MoldHoleLength+2*Const.MoldHoleLidBorderThickness)))
    gmsh.model.occ.translate([MoldHole1DimTag],0,Const.Height,-1.5*Const.Length)
    gmsh.model.occ.translate([MoldHoleLidDimTag],0,Const.Height+2*Const.MoldWallThickness,-1.5*Const.Length)
    FuseOut = gmsh.model.occ.fuse([MoldHole1DimTag],[MoldHoleLidDimTag])
    gmsh.model.occ.synchronize()
    gmsh.write("HoleLidForMoldStage2.step")
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("HoleLidForMoldStage2.stl")

    
#LidPG = gmsh.model.addPhysicalGroup(3,[LidDimTag])
    
def creakteMoldForCork():
 
    
    CavityCorkSketchDimTag = (2, FingerGeneration.createCavitySketch(Const.OuterRadius, Const.BellowHeight, Const.TeethRadius, Const.WallThickness, Const.CenterThickness, Const.PlateauHeight))
    
    CorkBellowHeight = Const.BellowHeight + 1.2
    CorkBellowThickness = 2*(Const.OuterRadius - Const.WallThickness) + 1
    
    FactorHeight = CorkBellowHeight/Const.BellowHeight
    FactorWidth = CorkBellowThickness/(2*(Const.OuterRadius-Const.WallThickness))

    gmsh.model.occ.dilate([CavityCorkSketchDimTag], 0, 0, 0, FactorWidth, 0, FactorHeight)
    
    print("FactorHeight: {}".format(FactorHeight))
    print("FactorWidht: {}".format(FactorWidth))
    
#    
#    CavityCorkSketchDimTag = (2,FingerGeneration.createCavitySketch(Const.OuterRadius, Const.NBellows, Const.BellowHeight, Const.TeethRadius, Const.WallThickness/2, Const.CenterThickness))
  
    ExtrudeDimTags = gmsh.model.occ.extrude([CavityCorkSketchDimTag],0,Const.CavityCorkThickness,0)
    
    HalfDimTag = ExtrudeDimTags[1]
    
    Tolerance = 2
    TotalBellowHeight = Const.NBellows * Const.BellowHeight
    CorkMoldHeight = TotalBellowHeight + 2 * Tolerance
    CorkMoldThickness = (Const.OuterRadius+Tolerance) * 2    
 
        
#    HalfCopyDimTag = gmsh.model.occ.copy([HalfDimTag])
#    print("HalfCopyDimTag: ", HalfCopyDimTag)
#    gmsh.model.occ.affineTransform(HalfCopyDimTag, [1,0,0,0, 0,1,0,0, 0,0,-1,0])
#     
#    FusionOut = gmsh.model.occ.fuse([HalfDimTag], HalfCopyDimTag)
#    CavityCorkDimTag = FusionOut[0][0]
    CavityCorkDimTag = HalfDimTag
    gmsh.model.occ.translate([CavityCorkDimTag],0,0,-TotalBellowHeight/2)
    
    CavityCork2DimTags = gmsh.model.occ.copy([CavityCorkDimTag])
    gmsh.model.occ.affineTransform(CavityCork2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    FuseOut = gmsh.model.occ.fuse([CavityCorkDimTag],CavityCork2DimTags)
    
    CompleteCorkDimTag = FuseOut[0][0]
    CavityMoldBoxDimTag = (3,gmsh.model.occ.addBox(-CorkMoldThickness/2,
                                                   -Tolerance,
                                                   -CorkMoldHeight/2,
                                                   CorkMoldThickness, 
                                                   Const.CavityCorkThickness+Tolerance,
                                                   CorkMoldHeight+Tolerance))
    CutOut = gmsh.model.occ.cut([CavityMoldBoxDimTag],[CompleteCorkDimTag])
    gmsh.model.occ.synchronize()
    gmsh.write("MoldForCork.step")
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("MoldForCork.stl")
    
    
def createFingerClamp():
    
#    gmsh.merge("Finger_Parametric.step")
#    FingerDimTags = gmsh.model.getEntities(3)
    FingerDimTags = [FingerGeneration.createFinger()]
    
    ClampBoxWidth = 4*Const.FixationWidth+Const.Thickness
    ClampBoxLength = 4*Const.FixationWidth
    ClampBoxHeight = Const.Height + 2 * Const.FixationWidth 
    ClampBoxDimDag = (3,gmsh.model.occ.addBox(-ClampBoxWidth/2,
                                           0,
                                           -ClampBoxLength/2,
                                           ClampBoxWidth,
                                           ClampBoxHeight,
                                           ClampBoxLength))
    ClampBoxCablePassDimTag = (3,gmsh.model.occ.addBox(-Const.Thickness/2, 
                                                       0,
                                                       0,
                                                       Const.Thickness,
                                                       5,
                                                       10
                                                       ))
    
    ScrewRadius = 1.7
    ScrewEarWidth = 6
    ScrewEarHeight = 3 
    ScrewEarLength = ScrewEarWidth
    
    ScrewEarBoxDimDag = (3,gmsh.model.occ.addBox(ClampBoxWidth/2,
                                           0,
                                           -ScrewEarLength/2,
                                           ScrewEarWidth,
                                           ScrewEarHeight,
                                           ScrewEarLength))
    
    ScrewLength = 6
    ScrewCylinderDimTag = (3,gmsh.model.occ.addCylinder(ClampBoxWidth/2+ScrewEarWidth/2,-ScrewLength/3,0, 0,ScrewLength,0,ScrewRadius))
    
    
    ScrewEarBox2DimTags = gmsh.model.occ.copy([ScrewEarBoxDimDag])
    gmsh.model.occ.affineTransform(ScrewEarBox2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    ScrewCylinder2DimTags = gmsh.model.occ.copy([ScrewCylinderDimTag])
    gmsh.model.occ.affineTransform(ScrewCylinder2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    CableHeight = 5*Const.Height/6
    CableLength = Const.LengthMold+2*Const.MoldWallThickness
    CableDimTag = (3,gmsh.model.occ.addCylinder(0,CableHeight,2*Const.MoldWallThickness,0,0,-CableLength,Const.CableRadius))
    
    FuseOut = gmsh.model.occ.fuse([ClampBoxDimDag],[ScrewEarBoxDimDag]+ScrewEarBox2DimTags)
    PositiveBoxDimTag = FuseOut[0][0]
    
    gmsh.model.occ.cut([PositiveBoxDimTag],FingerDimTags+ScrewCylinder2DimTags + [ScrewCylinderDimTag,CableDimTag]+[ClampBoxCablePassDimTag])
    gmsh.write("FingerClamp.step")
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("FingerClamp.stl")
    
#
#createMoldStage2()
#gmsh.clear()
#createMoldLidStage2()
#gmsh.clear()
#createHoleLidForMoldStage2()
#gmsh.clear()
creakteMoldForCork()
#gmsh.clear()
#createFingerClamp()
gmsh.model.occ.synchronize()
gmsh.fltk.run()
