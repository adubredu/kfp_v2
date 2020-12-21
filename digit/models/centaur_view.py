import pybullet as p 
import pybullet_data
from pybullet_planning import get_joint_limits, get_movable_joints


import time

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
# p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
p.loadURDF('floor/floor.urdf')

a1 = p.loadURDF('a1/a1.urdf',[0,0,0.4], useFixedBase=True)
# not_digit = p.loadURDF('not_digit/nl_digit.urdf',[0,0,0.6])

# cid = p.createConstraint(a1, -1, not_digit, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0], p.getQuaternionFromEuler([0, 1.5707, 0]))
for i in range(p.getNumJoints(a1)):
    print('')
    print(p.getJointInfo(a1, i))
joint_angles = [x[0] for x in p.getJointStates(a1, get_movable_joints(a1))]
print(joint_angles)
time.sleep(400)
