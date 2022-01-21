#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 11:30:30 2022

@author: stefan
"""

import numpy as np

# Geometric parameters
Length = 40
Height = 23
JointHeight = 6
Thickness = 20
JointSlopeAngle = np.deg2rad(30)
FixationWidth = 3

OuterRadius = Thickness/2 + 6
NBellowSteps = 3
StepHeight = 4
TeethRadius = Thickness/2   
WallThickness = 4
CenterThickness = 2
CavityCorkThickness = 3

# Elasticity parameters
PoissonRation = 0.3
YoungsModulus = 3000

# Mold parameters
MoldWallThickness = 3
MoldCoverTolerance = 0.1
