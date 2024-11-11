import pybullet as p
import pybullet_data
import os
import sys
import time
from icecream import ic

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
p.changeDynamics(plane, -1, lateralFriction=1.0)
model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jump_2.urdf")
model = p.loadURDF(model_path, [0, 0, 1], [0, 0, 0, 1])

for i in range(p.getNumJoints(model)):
    ic(p.getJointInfo(model, i))

while 1:
    p.stepSimulation()
    time.sleep(0.001)
