# gmsh-sofa_tutorial

## Requirements 
This tutorial/toolbox was tested with Gmsh 4.9.3 [1] and the SOFA v21.12 binaries including the SoftRobots-plugin (v21.12) for Linux and Windows. The binaries for SOFA and the SoftRobots-plugin can be found here [2]. Especially for Windows, don't forget to read the instructions [3].

[1] Install using pip ($pip install gmsh)

[2] https://www.sofa-framework.org/download/

[3] https://github.com/sofa-framework/sofa/releases/tag/v21.12.00

## First Steps

Before being able to launch any Scene, the corresponding geometries have to be created. For instance, for the finger, you need to run the corresponding script, i.e. "$python3 FingerGeneration.py", found in the folder Finger/Scenes/Geometries. In the same folder, the script "Constants.py" contains the parameters that describe the finger. After generation, the simulation can be launched, e.g. "$runSofa Finger_Direct.py" in the folder Finger/Scenes.

## Fabrication
