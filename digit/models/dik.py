import pybullet as p 
import pybullet_data
from pybullet_planning import get_joint_limits, multiply, get_link_pose,invert,get_movable_joints
from trac_ik_python.trac_ik import IK

import time

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
p.loadURDF('floor/floor.urdf')

# yumi = p.loadURDF('yumi_description/yumi_grippers.urdf', [1,1,0])
# base = p.loadURDF('buff_digit/panda_arm_hand.urdf',[0,0,0], useFixedBase=True)
base = p.loadURDF('buff_digit/digit.urdf',[0,0,0], useFixedBase=True)


for i in range(p.getNumJoints(base)):
    print('')
    print(p.getJointInfo(base, i))


time.sleep(3)
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




def setMotors(bodyId, joints,jointPoses):
    """
    Parameters
    ----------
    bodyId : int
    jointPoses : [float] * numDofs
    """
    assert(len(joints)==len(jointPoses))
    numJoints = p.getNumJoints(bodyId)

    for i,j in enumerate(joints):
        p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i])
        p.stepSimulation()

# setMotors(base, joint_pos)
	# p.stepSimulation()
def set_joint_motors(base, jointPoses):
    numJoints = p.getNumJoints(base)
    # for i in range(100):
    for i in range(numJoints):
        jointInfo = p.getJointInfo(base, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.setJointMotorControl2(bodyIndex=base, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[qIndex-7],targetVelocity=0, force=500, positionGain=0.03, velocityGain=0.5 )
            p.stepSimulation()

def solve_ik(urdf_path, pose):
    urdfstring = ''.join(open(urdf_path,'r').readlines())
    ik = IK('torso', 'left_gripper_ee',urdf_string=urdfstring)
    # ik = IK('panda_link0', 'panda_hand',urdf_string=urdfstring)

    init_joints = [0.0]*6
    (x,y,z),(rx,ry,rz,rw) = pose
    print(x,y,z)
    conf = ik.get_ik(init_joints, x,y,z,rx,ry,rz,rw)
    return conf

def pb_solve_ik(body, pose, ee_id):
    conf  = p.calculateInverseKinematics(body, ee_id, pose)
    return conf

def transform_pose(base, world_from_goal):
    #want torso_from_goal
    # tg = tw * wg 
    torso_from_world = invert(get_link_pose(base, 0))
    tg = multiply(torso_from_world, world_from_goal)
    return tg


def pb_ik_main():
    global base
    pose = ((0.3, -0.3,1),(0,0,0,1))
    ee_id = 16
    ree_id = 33
    conf = pb_solve_ik(base, pose[0],ree_id)
    # print('conf: ',conf)
    joints = get_movable_joints(base)
    # joints = [1,2,3,4,5,6]
    # set_joint_motors(base,conf)
    setMotors(base, joints, conf) 
    time.sleep(15)
    print('final pose: ',p.getLinkState(base, ree_id)[0])


def tracik_main():
    global base
    pose = ((0.3, -0.3,-0.3),(0,0,0,1))
    # pose = transform_pose(base, pose)
    # print('trans: ',pose)
    conf = solve_ik('buff_digit/digit_arm.urdf', pose)
    if conf is None:
        print('Cannot reach')
        return
    ee_id = 16

    # conf = pb_solve_ik(base, pose[0],ee_id)
    # print('len')
    print('conf: ',conf)
    # joints = [10,11,12,13,14,15]
    # joints = [1,2,3,4,5,6,8,9]
    joints = [1,2,3,4,5,6]
    # set_joint_motors(base,conf)
    setMotors(base, joints, conf) 
    time.sleep(10)
    print('final pose: ',p.getLinkState(base, ee_id)[0])


def joint_teleop():
    right_arm_joints = [1,2,3,4,5,6]
    ee_id = 16
    while True:
        print('final pose: ',p.getLinkState(base, ee_id)[0]);print('')
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



if __name__ == '__main__':
    pb_ik_main()
    # joint_teleop()
    # transform_pose(base, ((0.3,0,0.7),(0,0,0,1)))
    time.sleep(100)