import numpy as np
import pybullet as p
import pybullet_data
import pybullet_planning as pyplan
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
from utils.ikfast.digit_ik import solve_ik

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('../models')
p.loadURDF('floor/floor.urdf')
 
robot = p.loadURDF('buff_digit/prost_digit_freight.urdf',[0,0,0], useFixedBase=True)

left_joints = [4,5,9,11,12,13]
right_joints = [17,18,22,24,25,26]


def tf_arm_frame(robot, pose, basename): 
    baseid = pyplan.link_from_name(robot, basename)
    base_from_world = pyplan.invert(pyplan.get_link_pose(robot, baseid))
    tg = pyplan.multiply(base_from_world, pose)
    return tg

def set_joint_motors(bodyId, joints,jointPoses): 
    assert(len(joints)==len(jointPoses)) 

    for i,j in enumerate(joints):
        p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i])
        p.stepSimulation()
        time.sleep(0.25)
time.sleep(5)
lpose = ((0.5,0.5,1),(0,0,0,1))
rpose = ((0.5,-0.5,1),(0,0,0,1))

lrel_pose = tf_arm_frame(robot, lpose, 'left_jlink0')
rrel_pose = tf_arm_frame(robot, rpose, 'right_jlink0') 
print(lrel_pose)
print(rrel_pose)
lgen = solve_ik(lrel_pose[0], lrel_pose[1],'left_arm')
rgen = solve_ik(rrel_pose[0], rrel_pose[1],'right_arm')

# print('left base: ',pyplan.get_link_pose(robot, pyplan.link_from_name(robot, 'left_gripper_ee')))
# print('right base: ',pyplan.get_link_pose(robot, pyplan.link_from_name(robot, 'right_gripper_ee')))
 
# '''
for a,b in zip(lgen,rgen): 
	set_joint_motors(robot, left_joints, a)
	time.sleep(5)
	print('lee_pose: ',pyplan.get_link_pose(robot, pyplan.link_from_name(robot, 'left_gripper_ee')))
	print('-'*15)
	print(' ')

	set_joint_motors(robot, right_joints, b)
	time.sleep(5)
	print('ree_pose: ',pyplan.get_link_pose(robot, pyplan.link_from_name(robot, 'right_gripper_ee')))
	print('-'*15)
	print(' ')
# '''
time.sleep(100)