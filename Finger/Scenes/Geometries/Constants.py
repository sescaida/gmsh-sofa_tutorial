#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 11:30:30 2022

@author: stefan
"""

import numpy as np

def filterDimTagz(DimTagz, dimension=0):
    FilteredTagz = []
    for DimTag in DimTagz:
        if DimTag[0] == dimension:
            FilteredTagz.append(DimTag)
def dimTagz2Tagz(DimTagz, dimension):
    Tagz = [tag for (dim, tag) in DimTagz]
    return Tagz
        
# Geometric parameters
Length = 40
Height = 20
#JointHeight = 6
JointHeight = 6.26
Thickness = 17.5
JointSlopeAngle = np.deg2rad(30)
#JointSlopeAngle = 0.3376
FixationWidth = 3

OuterRadius = Thickness/2 + 6 #max value is Thickness/2 + 9! 
#OuterRadius = 9.09 
NBellows = 1
BellowHeight = 12
TeethRadius = Thickness/2   
WallThickness = 3.5
#CenterThickness = 1.5
CenterThickness = 2.737
CavityCorkThickness = 3
#CavityCorkThickness = 4.805
PlateauHeight = 3

# Elasticity parameters
PoissonRation = 0.3
YoungsModulus = 3000

# Mold parameters
MoldWallThickness = 3
MoldCoverTolerance = 0.2
LengthMold = 3*Length + 2*MoldWallThickness
LidHoleBorderThickness = 1
LidHoleThickness = Thickness - 2*LidHoleBorderThickness
LidHoleLength = 3*Length/5

MoldHoleThickness = Thickness - 2*LidHoleBorderThickness
MoldHoleLength = Length/2

ThicknessMold = 2*OuterRadius + 2*MoldWallThickness
LengthMold = 3*Length + 2*MoldWallThickness
HeightMold = Height + FixationWidth + MoldWallThickness    
MoldHoleLidBorderThickness = 2

#Cable
CableRadius = 0.8
CableDistance = 10
#CableHeight = 17.75
CableHeight = OuterRadius + 1.2
