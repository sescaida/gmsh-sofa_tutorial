#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 13 12:10:07 2022

@author: stefan
"""

import gmsh
import numpy as np
import locale
import Constants
locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')

gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 1)


def defineMeshSizes(lc=0.5):   
    #-------------------
    # MeshSizes 
    #-------------------

    gmsh.model.mesh.field.add("Box", 6)
    gmsh.model.mesh.field.setNumber(6, "VIn", lc)
    gmsh.model.mesh.field.setNumber(6, "VOut", lc)
    gmsh.model.mesh.field.setNumber(6, "XMin", -Constants.Thickness)
    gmsh.model.mesh.field.setNumber(6, "XMax", Constants.Thickness)
    gmsh.model.mesh.field.setNumber(6, "YMin", 0)
    gmsh.model.mesh.field.setNumber(6, "YMax", Constants.Height)
    gmsh.model.mesh.field.setNumber(6, "ZMin", -3*Constants.Length)
    gmsh.model.mesh.field.setNumber(6, "ZMax", 0)    
    gmsh.model.mesh.field.setNumber(6, "Thickness", 0.3)
     
    gmsh.model.mesh.field.setAsBackgroundMesh(6)
    
    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    

def createLines(PointTags):
    
    LineTags = np.empty((0,1),dtype=int)
    NPoints = len(PointTags)
    for i in range(1, NPoints):
        LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[i-1], PointTags[i])])
    
    LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[-1], PointTags[0])])
    
    #print('LineTags: ' + str(LineTags))
    return LineTags 

def makeSegmentTipMod(SegmentDimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, FixationWidth=3, lc=1):
     
    Point1Tag = gmsh.model.occ.addPoint(-Thickness/2,JointHeight,-Length)
    
    LengthDiagonal = (Height-JointHeight)/np.cos(JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(JointSlopeAngle)
    Point2Tag = gmsh.model.occ.addPoint(-Thickness/2,Height,JointStandoff-Length)
    CenterPointTag = gmsh.model.occ.addPoint(-Thickness/2,0,-Length/2)
    CircleArcTag = gmsh.model.occ.addCircleArc(Point1Tag, CenterPointTag, Point2Tag)
    
    pass  

def makeSegmentStage1Mod(SegmentDimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, lc=1):
    
    LengthDiagonal = (Height-JointHeight)/np.cos(JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(JointSlopeAngle)    
    Stage1DistanceToStandoff = 2
    Stage1DistanceFromBase = 5
    Overboarding = 3
    SubstractionDepth = 2.5
    ZOffset = JointStandoff+Stage1DistanceToStandoff
    BoxSubstractionOuterDimTag = (3,gmsh.model.occ.addBox(-Thickness/2-Overboarding, 
                                                          Stage1DistanceFromBase,
                                                          -ZOffset, 
                                                          Thickness+2*Overboarding, 
                                                          Height+Overboarding,
                                                          -(Length-2*ZOffset)))
    
    BoxSubstractionInnerDimTag = (3,gmsh.model.occ.addBox(-Thickness/2+SubstractionDepth, 
                                                          Stage1DistanceFromBase,
                                                          -ZOffset,
                                                          Thickness-2*SubstractionDepth,
                                                          Height-(SubstractionDepth+Stage1DistanceFromBase),
                                                          -(Length-2*ZOffset)))
    
    CutOut = gmsh.model.occ.cut([BoxSubstractionOuterDimTag], [BoxSubstractionInnerDimTag])
    SubstractionObjectDimTag = CutOut[0][0]
    
    CutOut = gmsh.model.occ.cut([SegmentDimTag],[SubstractionObjectDimTag])
    Segment1ModDimTag = CutOut[0][0]
    return Segment1ModDimTag
    
def makeSegmentFixationMod(SegmentDimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, FixationWidth, lc=1):
    
    BoxFillDimTag = gmsh.model.occ.addBox(-Thickness/2,
                                          0,
                                          0,
                                          Thickness,
                                          Height,
                                          -Length/2)
    
    BoxFixationDimTag = gmsh.model.occ.addBox(-(Thickness/2+FixationWidth),
                                              0,
                                              0,
                                              Thickness+2*FixationWidth, 
                                              Height+FixationWidth,
                                              -FixationWidth)
    
    BoxesDimTags = [(3,BoxFillDimTag),(3,BoxFixationDimTag)]
    FuseOut = gmsh.model.occ.fuse([SegmentDimTag],BoxesDimTags)
    SegmentDimTag = FuseOut[0][0]
    return SegmentDimTag
    
def createSegment(Length, Height, JointHeight, Thickness, JointSlopeAngle,lc=1):
    
    PointTags = np.empty((0,1), int)
    
    LengthDiagonal = (Height-JointHeight)/np.cos(JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(JointSlopeAngle)
    
    YValues = np.array([0,0,JointHeight,Height,Height,JointHeight])
    ZValues = np.array([0,-Length,-Length, -Length+JointStandoff,-JointStandoff,0])
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(Thickness/2, YValue, ZValue, lc) for (YValue,ZValue) in zip(YValues,ZValues)])
    
    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])
    ExtrudeTags = gmsh.model.occ.extrude([(2,SurfaceTag)],-Thickness,0,0)
    print("Segment extrude dim tags:", ExtrudeTags)
    gmsh.model.occ.synchronize()
    
    return ExtrudeTags[1]

def createArticulationBellow(OuterRadius, NBellows, BellowHeight, TeethRadius, FingerWidth, PlateauHeight, lc=1):
    
    PointTags = np.empty((0,1), int)
    
    DiagonalStepHeight = (BellowHeight - PlateauHeight)/2
    TotalBellowHeight = NBellows*BellowHeight
    
    XValueIn = TeethRadius     
    XValueOut = OuterRadius 
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight+PlateauHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, BellowHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, BellowHeight, lc)])    

    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])                
    
    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,0,1, np.pi)
    SingleBellowDimTag = RevolveDimTags[1]    

    Copies = []
    for i in range(1,NBellows):
        CopyDimTags = gmsh.model.occ.copy([SingleBellowDimTag])
        gmsh.model.occ.translate(CopyDimTags,0,0,i*BellowHeight)
        Copies.append(CopyDimTags[0])
    
 
    FuseOut = gmsh.model.occ.fuse([SingleBellowDimTag], Copies)
    BellowDimTags = FuseOut[0]    
    print("BellowDimTag: {}".format(BellowDimTags))
    gmsh.model.occ.translate(BellowDimTags,0,0,-TotalBellowHeight/2)
#    gmsh.model.occ.synchronize()
#    gmsh.fltk.run()
#    exit()   
    return BellowDimTags

def createCavitySketch(OuterRadius, BellowHeight, TeethRadius, WallThickness, CenterThickness, PlateauHeight, lc=1 ):
    
    
    PointTags = np.empty((0,1), int)
    
    DiagonalStepHeight = (BellowHeight - PlateauHeight)/2
    
    XValueIn = TeethRadius - WallThickness
    print("XValueIn: {}".format(XValueIn))
    XValueOut = OuterRadius - WallThickness
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight+PlateauHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, BellowHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, BellowHeight, lc)])    

    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])            
    return SurfaceTag
    
def createCavityVolume(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, lc=1):
    
    TotalHeight = NBellows * BellowHeight
    SurfaceTag = createCavitySketch(OuterRadius, BellowHeight, TeethRadius, WallThickness, CenterThickness, Constants.PlateauHeight)
    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,0,1, np.pi)
    CavityDimTag = RevolveDimTags[1]
    
    Copies = []
    for i in range(1,NBellows):        
        HalfCopyDimTag = gmsh.model.occ.copy([CavityDimTag])
        Copies.append(HalfCopyDimTag[0])
        gmsh.model.occ.translate(HalfCopyDimTag, 0,0,i*BellowHeight)         
    
    FusionOut = gmsh.model.occ.fuse([CavityDimTag], Copies)
      
    CavityBaseDimTag = FusionOut[0]
    
    gmsh.model.occ.translate(CavityBaseDimTag,0,0,-TotalHeight/2)
    
    BoxDimTag = (3,gmsh.model.occ.addBox(-OuterRadius,
                                    0,
                                    -(TotalHeight+1),
                                    2*OuterRadius, 
                                    CavityCorkThickness, 
                                    2*(TotalHeight+1)))    
        
    CutOut = gmsh.model.occ.cut(CavityBaseDimTag,[BoxDimTag])
    print("CutOut: ", CutOut)
    CavityDimTags = CutOut[0]
    
#    gmsh.model.occ.synchronize()
#    gmsh.fltk.run()
    return CavityDimTags

def createAllCavities(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness,lc=1):
    
    BellowGap = (NBellows-1)*BellowHeight
    Cavity1DimTags = createCavityVolume(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, Constants.CavityCorkThickness, lc=lc)
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-(Constants.Length+BellowGap/2))
    Cavity2DimTags = gmsh.model.occ.copy(Cavity1DimTags)        
    
    gmsh.model.occ.translate(Cavity2DimTags,0,0,-(Constants.Length+BellowGap))
    
    gmsh.model.occ.synchronize()
    AllCavitiesDimTags = Cavity1DimTags + Cavity2DimTags 
    return AllCavitiesDimTags

def exportCavities(lc=5):
    #-------------------
    # Segments 
    #-------------------
    gmsh.model.add("Cavity")            

    #-------------------
    # Cavities
    #-------------------   
    lc = 0.5
    
    Cavity1DimTags = createCavityVolume(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, Constants.CavityCorkThickness, lc=lc)
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-Constants.Length)
    gmsh.model.occ.synchronize()
    defineMeshSizes(lc)   
    gmsh.model.mesh.generate(2)        
    gmsh.write("Cavity01.stl")
    
    gmsh.clear()
    
    Cavity1DimTags = createCavityVolume(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, Constants.CavityCorkThickness, lc=lc)    
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-Constants.Length)
    gmsh.model.occ.affineTransform(Cavity1DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.synchronize()
    defineMeshSizes(lc)   
    gmsh.model.mesh.generate(2)    
    
    gmsh.write("Cavity02.stl")
    
    gmsh.clear()
    
    Cavity1DimTags = createCavityVolume(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, Constants.CavityCorkThickness, lc=lc)
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-2*Constants.Length)
    gmsh.model.occ.synchronize()
    defineMeshSizes(lc)   
    gmsh.model.mesh.generate(2)    
    gmsh.write("Cavity03.stl")
    
    gmsh.clear()
    
    Cavity1DimTags = createCavityVolume(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, Constants.CavityCorkThickness, lc=lc)    
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-2*Constants.Length)
    gmsh.model.occ.affineTransform(Cavity1DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.synchronize()
    defineMeshSizes(lc)   
    gmsh.model.mesh.generate(2)    
    gmsh.write("Cavity04.stl")    
    
    gmsh.clear()
    
def createFinger(Stage1Mod=False, lc = 7):
    #-------------------
    # Segments 
    #-------------------            
    
    Segment1DimTag = createSegment(Constants.Length, Constants.Height, Constants.JointHeight, Constants.Thickness, Constants.JointSlopeAngle)        
    Segment2DimTags = gmsh.model.occ.copy([Segment1DimTag])
    if Stage1Mod:
        Segment2DimTags = [makeSegmentStage1Mod(Segment2DimTags[0], Constants.Length, Constants.Height, Constants.JointHeight, Constants.Thickness, Constants.JointSlopeAngle, lc=1)]    
    Segment3DimTags = gmsh.model.occ.copy(Segment2DimTags)
    Segment1DimTag = makeSegmentFixationMod(Segment1DimTag, Constants.Length, Constants.Height, Constants.JointHeight, Constants.Thickness, Constants.JointSlopeAngle, Constants.FixationWidth)    

#    gmsh.model.occ.synchronize()
#    gmsh.fltk.run()
#       
    BellowGap = Constants.BellowHeight*(Constants.NBellows-1)
    gmsh.model.occ.translate(Segment2DimTags,0,0,-(BellowGap+Constants.Length))
    gmsh.model.occ.translate(Segment3DimTags,0,0,-2*(BellowGap+Constants.Length))    
    
    #-------------------
    # Bellows
    #-------------------           
    
    Bellow1DimTags = createArticulationBellow(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.Thickness, Constants.PlateauHeight, lc=lc)
    Bellow2DimTags = gmsh.model.occ.copy(Bellow1DimTags)

    gmsh.model.occ.translate(Bellow1DimTags,0,0,-(BellowGap/2+Constants.Length))
    gmsh.model.occ.translate(Bellow2DimTags,0,0,-2*(3*BellowGap/4+Constants.Length))
    
    FuseOut = gmsh.model.occ.fuse(Bellow1DimTags,[Segment1DimTag]+Segment2DimTags + Segment3DimTags + Bellow2DimTags)
    FingerNoCavitiesDimTag = FuseOut[0]
    gmsh.model.occ.synchronize()
    
    #-------------------
    # Cavities
    #-------------------
    
    AllCavitiesDimTags = createAllCavities(Constants.OuterRadius, Constants.NBellows, Constants.BellowHeight, Constants.TeethRadius, Constants.WallThickness, Constants.CenterThickness, lc=lc)
   

    print("AllCavitiesDimTags: ", AllCavitiesDimTags)
    #-------------------
    # Cut Cavities
    #-------------------
    
    CutOut = gmsh.model.occ.cut(FingerNoCavitiesDimTag,AllCavitiesDimTags)
    FingerDimTag = CutOut[0][0]
    gmsh.model.occ.synchronize()
 
    #-------------------
    # Export 
    #-------------------
    
    defineMeshSizes(lc)
    
    gmsh.model.occ.synchronize()    
    gmsh.write("Finger_Parametric.step")
    gmsh.model.mesh.generate(3)
    gmsh.write("Finger_Volumetric.vtk")
    
    gmsh.model.mesh.clear()
    #gmsh.model.mesh.generate(0)
    gmsh.model.mesh.generate(2)
    gmsh.model.mesh.refine()
    gmsh.write("Finger_Surface.stl")

    return FingerDimTag
 
 
def createShapes():
    exportCavities()
    FingerDimTag = createFinger(Stage1Mod=False)

if __name__ == '__main__':
    createShapes()
    gmsh.fltk.run()