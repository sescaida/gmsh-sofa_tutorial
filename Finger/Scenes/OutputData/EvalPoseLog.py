#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 17:27:51 2022

@author: stefan
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


PoseLog = np.loadtxt("PoseLog.txt")

OptiTrackPoses = PoseLog[:,:7]
OptiTrackPositions = OptiTrackPoses[:,:3]
OptiTrackOrientations = R.from_quat(OptiTrackPoses[:,3:])

SOFAPoses = PoseLog[:,7:]
SOFAPositions = SOFAPoses[:,:3]
SOFAOrientations = R.from_quat(SOFAPoses[:,3:])


InitOTPose = OptiTrackPoses[0,:]
InitOTPosition = InitOTPose[0:3]
InitOTOrientation = R.from_quat(InitOTPose[3:])

CleanedOTPositions = OptiTrackPositions - InitOTPosition
CleanedOTOrientations = InitOTOrientation.inv() * OptiTrackOrientations
CleanedOTOrientationAxisAngle = CleanedOTOrientations.as_rotvec()
OTAngles = np.rad2deg(np.linalg.norm(CleanedOTOrientationAxisAngle, axis=1))


InitSOFAPose = SOFAPoses[0,:]
InitSOFAPosition = InitSOFAPose[0:3]
InitSOFAOrientation = R.from_quat(InitSOFAPose[3:])

CleanedSOFAPositions = SOFAPositions-InitSOFAPosition
CleanedSOFAOrientations = InitSOFAOrientation.inv() * SOFAOrientations
CleanedSOFAOrientationAxisAngle = CleanedSOFAOrientations.as_rotvec()
SOFAAngles = np.rad2deg(np.linalg.norm(CleanedSOFAOrientationAxisAngle, axis=1))


DistancesFromOriginOT = np.linalg.norm(CleanedOTPositions, axis=1)
MaxDistFromOriginOT = np.max(DistancesFromOriginOT)

ErrorsAbsolute = np.linalg.norm(CleanedSOFAPositions - CleanedOTPositions,axis=1)

ErrorsPercentage = ErrorsAbsolute/MaxDistFromOriginOT * 100

for i in range(len(CleanedOTOrientations)):
    DiffOrientation  = CleanedOTOrientations[i,:].inv() * CleanedSOFAOrientations[i,:]
    AxisAngle = DiffOrientation.as_rotvec()
    Angle = np.linalg.norm(AxisAngle)
    print("Diff Angle: {}".format(np.rad2deg(Angle)))
