import pybullet as p 
import pybullet_data
from pybullet_planning import get_joint_limits


import time

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
p.loadURDF('floor/floor.urdf')

# yumi = p.loadURDF('yumi_description/yumi_grippers.urdf', [1,1,0])
base = p.loadURDF('not_digit/not_digit.urdf',[0,0,0.9], useFixedBase=True)
# joint_pos = p.calculateInverseKinematics(base, 13,[0.5,0.1,0.7] )

right_arm_joints = [8,9,10,11,12,13]
# print(len(joint_pos))
# print(p.getNumJoints(base))
for i in range(p.getNumJoints(base)):
    print('')
    print(p.getJointInfo(base, i))
    # qIndex = j[3]
    # if qIndex > -1:
    #     print(j)
	# p.setJointMotorControl2(base, i,
				# controlMode=p.POSITION_CONTROL,targetPosition=joint_pos[i],
				# force=3000)

# yumi = p.loadURDF('yumi_description/yumi_grippers.urdf', [0.2,0,1])

# lift_index=7
# cid = p.createConstraint(base, lift_index, yumi, -1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0., 0., 0])

time.sleep(500)
def control_joint(joint, value, minn, maxx):
    global base
    minn, maxx = get_joint_limits(base, joint)
    if value < minn:
        value = minn
    if value > maxx:
        value = maxx
    p.setJointMotorControl2(base, joint,
				controlMode=p.POSITION_CONTROL,targetPosition=value,
				force=3000)




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
    if ord('a') in keys:
        joint = right_arm_joints[0]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('s') in keys:
        joint = right_arm_joints[0]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('z') in keys:
        joint = right_arm_joints[1]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('x') in keys:
        joint = right_arm_joints[1]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)
    if ord('b') in keys:
        joint = right_arm_joints[2]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('n') in keys:
        joint = right_arm_joints[2]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)
    if ord('i') in keys:
        joint = right_arm_joints[3]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('o') in keys:
        joint = right_arm_joints[3]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)
    if ord(',') in keys:
        joint = right_arm_joints[4]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('.') in keys:
        joint = right_arm_joints[4]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord('l') in keys:
        joint = right_arm_joints[5]
        angle = p.getJointState(base, joint)[0]
        angle -= delta
        control_joint(joint, angle, 0.0, 0.3)

    if ord(';') in keys:
        joint = right_arm_joints[5]
        angle = p.getJointState(base, joint)[0]
        angle += delta
        control_joint(joint, angle, 0.0, 0.3)
# time.sleep(1)
# print(p.getLinkState(base, 13)[0])
# time.sleep(100)