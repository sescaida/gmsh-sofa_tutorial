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

def hide_all():
    ent = gmsh.model.getEntities()
    for x in ent:
        #print("x: ", x)
        gmsh.model.setVisibility((x,), False)


def createFingerMold(Stage1Mod=False):        
    
    FingerDimTag = FingerGeneration.createFinger(Stage1Mod)
    
    MoldBoxDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2,
                                             0,
                                             Const.MoldWallThickness, 
                                             Const.ThicknessMold, 
                                             Const.HeightMold, 
                                             -Const.LengthMold))
    
    CableHeight = 5*Const.Height/6
    CableLength = Const.LengthMold+2*Const.MoldWallThickness
    CableDimTag = (3,gmsh.model.occ.addCylinder(0,CableHeight,2*Const.MoldWallThickness,0,0,-CableLength,Const.CableRadius))
    
    #gmsh.fltk.run()
    CutOut = gmsh.model.occ.cut([MoldBoxDimTag],[FingerDimTag, CableDimTag])
    
    MoldBaseDimTag = CutOut[0][0]
    AllCavitiesDimTags = CutOut[0][1:]
    
    print("MoldBaseDimTag : ", MoldBaseDimTag )
    print("AllCavities: ", AllCavitiesDimTags)
    
    MoldBoxOuterRimDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2,
                                                     0,
                                                     Const.MoldWallThickness, 
                                                     Const.ThicknessMold, 
                                                     -Const.MoldWallThickness, 
                                                     -Const.LengthMold))
     
    MoldBoxInnerRimDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2+Const.MoldWallThickness,
                                                     0,
                                                     0,
                                                     Const.ThicknessMold-2*Const.MoldWallThickness,
                                                     -Const.MoldWallThickness, 
                                                     -Const.LengthMold+2*Const.MoldWallThickness))
    
    CutOut = gmsh.model.occ.cut([MoldBoxOuterRimDimTag],[MoldBoxInnerRimDimTag])
    MoldRim = CutOut[0][0]
    
    FuseOut = gmsh.model.occ.fuse([MoldBaseDimTag],[MoldRim])
    MoldDimTag = FuseOut[0][0]
    #MoldPG = gmsh.model.addPhysicalGroup(3,[MoldDimTag])
    
    gmsh.model.occ.synchronize()
    
    return MoldDimTag, AllCavitiesDimTags

def createMoldLid(AllCavitiesDimTags):
    
    #-----------------
    # Create mold lid
    #-----------------
    MoldLidTopDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2,
                                                -Const.MoldWallThickness,
                                                Const.MoldWallThickness, 
                                                Const.ThicknessMold, 
                                                -Const.MoldWallThickness, 
                                                -Const.LengthMold))
    
    MoldLidInteriorDimTag = (3,gmsh.model.occ.addBox(-Const.ThicknessMold/2+Const.MoldWallThickness+Const.MoldCoverTolerance,
                                                     0,
                                                     Const.MoldCoverTolerance, 
                                                     Const.ThicknessMold-2*Const.MoldWallThickness-2*Const.MoldCoverTolerance,
                                                     -Const.MoldWallThickness, 
                                                     -Const.LengthMold+2*Const.MoldWallThickness+2*Const.MoldCoverTolerance))
    
    #-----------------
    # Create cavity cork
    #-----------------
    
    CavityCorkSketchDimTag = (2,FingerGeneration.createCavitySketch(Const.OuterRadius, Const.NBellowSteps, Const.StepHeight, Const.TeethRadius, Const.WallThickness/2, Const.CenterThickness))
    
    ExtrudeDimTags = gmsh.model.occ.extrude([CavityCorkSketchDimTag],0,Const.CavityCorkThickness,0)
    
    HalfDimTag = ExtrudeDimTags[1]
        
    HalfCopyDimTag = gmsh.model.occ.copy([HalfDimTag])
    print("HalfCopyDimTag: ", HalfCopyDimTag)
    gmsh.model.occ.affineTransform(HalfCopyDimTag, [1,0,0,0, 0,1,0,0, 0,0,-1,0])
     
    FusionOut = gmsh.model.occ.fuse([HalfDimTag], HalfCopyDimTag)
    CavityCorkDimTag = FusionOut[0]
    
    gmsh.model.occ.translate(CavityCorkDimTag,0,0,-Const.Length)
    CavityCork2DimTags = gmsh.model.occ.copy(CavityCorkDimTag)
    CavityCork3DimTags = gmsh.model.occ.copy(CavityCorkDimTag)
    CavityCork4DimTags = gmsh.model.occ.copy(CavityCorkDimTag)    
    
    gmsh.model.occ.affineTransform(CavityCork2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.translate(CavityCork3DimTags,0,0,-Const.Length)
    gmsh.model.occ.affineTransform(CavityCork4DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.translate(CavityCork4DimTags,0,0,-Const.Length)
    gmsh.model.occ.synchronize()
    AllCavitiesCorkDimTags = CavityCorkDimTag + CavityCork2DimTags + CavityCork3DimTags + CavityCork4DimTags
    
    FuseOut = gmsh.model.occ.fuse([MoldLidTopDimTag],[MoldLidInteriorDimTag]+AllCavitiesCorkDimTags+AllCavitiesDimTags)
    LidDimTag = FuseOut[0][0]
    return LidDimTag

#LidPG = gmsh.model.addPhysicalGroup(3,[LidDimTag])



def createMoldParts():    
    
    MoldDimTag, AllCavitiesDimTags = createFingerMold(Stage1Mod=True)
    LidDimTag = createMoldLid(AllCavitiesDimTags)
    hide_all()
    gmsh.model.setVisibility((LidDimTag,),False, True)
    gmsh.model.setVisibility((MoldDimTag,),False, True)    
    print("MoldDimTag: ", MoldDimTag)
    print("LiddDimTag: ", LidDimTag)
    gmsh.write("Mold.step")    
    gmsh.model.occ.synchronize()    
    return MoldDimTag, LidDimTag
    #gmsh.fltk.run()    
#    gmsh.model.mesh.generate(2)
#    gmsh.model.mesh.refine()
#    gmsh.write("Mold.stl")    
#    
    
if __name__ == "__main__":
    createMoldParts()
    gmsh.fltk.run()