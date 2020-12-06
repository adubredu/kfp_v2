import pybullet as p 
import pybullet_data

import time

direct = p.connect(p.GUI)  
delta = 0.001
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath('models')
p.loadURDF('floor/floor.urdf')


base = p.loadURDF('fetch_description/robots/freight.urdf')
for i in range(p.getNumJoints(base)):
	print(p.getJointInfo(base, i))
while True:
	time.sleep(100)
	pass