#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 11:30:30 2022

@author: stefan
"""

import numpy as np

# Geometric parameters
Length = 40
Height = 20
JointHeight = 6
Thickness = 17.5
JointSlopeAngle = np.deg2rad(30)
FixationWidth = 3

OuterRadius = Thickness/2 + 6
NBellowSteps = 1
StepHeight = 4
TeethRadius = Thickness/2   
WallThickness = 4
CenterThickness = 1.5
CavityCorkThickness = 3

# Elasticity parameters
PoissonRation = 0.3
YoungsModulus = 3000

# Mold parameters
MoldWallThickness = 3
MoldCoverTolerance = 0.1
ThicknessMold = 2*Const.OuterRadius + 2*Const.MoldWallThickness
LengthMold = 3*Const.Length + 2*Const.MoldWallThickness
HeightMold = Const.Height + Const.FixationWidth + Const.MoldWallThickness    