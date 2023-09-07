import pybullet as p
from time import sleep
import pybullet_data

import numpy as np
from datetime import datetime

#p.DIRECT for without gui
options = "--background_color_red=1 --background_color_blue=1 --background_color_green=1"
physicsClient = p.connect(p.GUI, options=options) #p.DIRECT)#

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf", [0,0,0])
cup = p.loadURDF("cup.urdf", basePosition=[0,0,0.8], 
	baseOrientation = [1,0,0,1], globalScaling=0.5, useMaximalCoordinates = True)
#cup = p.loadURDF("cup.urdf", [0,0,0.8], globalScaling=0.5)


water = []
'''
for i in range(10):
	for j in range(10):
		for k in range(2):
			pos = [-0.2+(i/40),-0.2+(j/40),0.4+(k/40)]
			water.append(p.loadURDF("water.urdf", basePosition=pos))
	pass
'''


for i in range(10):
	pos = [-0.2+np.sin(i)/3,-np.cos(i)/3,0.2+(i/800)]
	water.append(p.loadURDF("water.urdf", pos))

p.setRealTimeSimulation(1)

while p.isConnected():
	sleep(0.1)  # Time in seconds.	
	p.stepSimulation()	

