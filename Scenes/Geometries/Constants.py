#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 11:30:30 2022

@author: stefan
"""

import numpy as np

# Geometric parameters
Length = 50
Height = 25
JointHeight = 8
Thickness = 25
JointSlopeAngle = np.deg2rad(30)

OuterRadius = Thickness/2 + 7
NBellowSteps = 3
StepHeight = 5
TeethRadius = Thickness/2   
WallThickness = 5
CenterThickness = 3

# Elasticity parameters
PoissonRation = 0.3
YoungsModulus = 3000