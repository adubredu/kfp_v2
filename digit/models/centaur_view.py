import pybullet as p 
import pybullet_data
from pybullet_planning import get_joint_limits, get_movable_joints, set_pose,Pose, Point, stable_z, get_side_cylinder_grasps, get_pose


import time

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
# p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
p.loadURDF('floor/floor.urdf')

base = p.loadURDF('a1/a1.urdf',[0,0,0.4], useFixedBase=True)
table1 = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
table2 = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=True)
# pepsi = p.loadSDF('can_pepsi/model.sdf')
# pepsi = pepsi[0]
block = p.loadURDF('block/block.urdf')
set_pose(block, Pose(Point(x=0.7, z=stable_z(block, table1))))
block_pose = get_pose(block)
print(block_pose)
# grasps = get_side_cylinder_grasps(block)
# for g in grasps:
# 	g_pose = g 
# 	break


# not_digit = p.loadURDF('not_digit/nl_digit.urdf',[0,0,0.6])

# cid = p.createConstraint(a1, -1, not_digit, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0], p.getQuaternionFromEuler([0, 1.5707, 0]))
# for i in range(p.getNumJoints(a1)):
#     print('')
#     print(p.getJointInfo(a1, i))
# joint_angles = [x[0] for x in p.getJointStates(a1, get_movable_joints(a1))]
# print(joint_angles)
# time.sleep(400)

def control_joint(joint, value, minn, maxx):
    global base
    # minn, maxx = get_joint_limits(base, joint)
    if value < minn:
        value = minn
    if value > maxx:
        value = maxx
    p.setJointMotorControl2(base, joint,
				controlMode=p.POSITION_CONTROL,targetPosition=value,
				force=3000)
    p.stepSimulation()




def setMotors(bodyId, jointPoses):
    """
    Parameters
    ----------
    bodyId : int
    jointPoses : [float] * numDofs
    """
    numJoints = p.getNumJoints(bodyId)

    for i in range(numJoints):
        jointInfo = p.getJointInfo(bodyId, i)
        #print(jointInfo)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[qIndex-7])
        

# setMotors(base, joint_pos)
	# p.stepSimulation()
while True:
    keys = p.getKeyboardEvents()
    if ord('z') in keys:
        control_joint(16, 0.025, 0.0, 0.3)
        control_joint(17, 0.025, 0.0, 0.3)

    if ord('x') in keys:
        control_joint(16, 0.0, 0.0, 0.3)
        control_joint(17, 0.0, 0.0, 0.3)