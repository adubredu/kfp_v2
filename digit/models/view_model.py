import pybullet as p  

import pybullet_data
from pybullet_planning import get_joint_limits, multiply, get_link_pose,invert,get_movable_joints
import pybullet_planning as pyplan 
import time,sys
from matplotlib import pyplot as plt 
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

   
delta = 0.01
 
client_id = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
# p.setGravity(0, 0, -9.81)
# p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
# p.loadURDF('floor/floor.urdf') 
# base = p.loadURDF('buff_digit/prost_digit_hover.urdf',[0,0,0], useFixedBase=True)
# base = p.loadURDF('nice_kitchen/nice_kitchen.urdf', [0,0,0],p.getQuaternionFromEuler([-1.57,3.1415,0]), useFixedBase=True,flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL) 
base = p.loadURDF('table/table.urdf', [0,0,0], useFixedBase=True ) 

# base = p.loadSDF('bottle_beer/model.sdf')
# time.sleep(100000)
right_arm_joints = [17,18,22,24,25,26]
left_arm_joints = [4,5,9,11,12,13]
right_rest_conf = [-1.3587102702612153, -0.9894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
left_rest_conf = [-1.3587102702612153, 0.9894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
# print(len(joint_pos))
# print(p.getNumJoints(base))
for i in range(p.getNumJoints(base)):
    print('')
    print(p.getJointInfo(base, i))
print(get_movable_joints(base))

while 1:
    # viewMatrix = p.computeViewMatrix(
    #             cameraEyePosition=[0, -1., 2],
    #             cameraTargetPosition=[0, 0, 0],
    #             cameraUpVector=[0, 1, 0])

    # projectionMatrix = p.computeProjectionMatrixFOV(
    #     fov=60.0,
    #     aspect=1.0,
    #     nearVal=0.02,
    #     farVal=3.1)

    # width, height, colorImage, depthImg, segImg = p.getCameraImage(
    #     width=640, 
    #     height=640,
    #     viewMatrix=viewMatrix,
    #     projectionMatrix=projectionMatrix,
    #     shadow=True, renderer=p.ER_TINY_RENDERER )
    # plt.imshow(colorImage) 
    # plt.show()
    p.stepSimulation()

#print(pyplan.get_link_pose(base,pyplan.link_from_name(base, 'right_gripper_ee'))) 
#print(pyplan.get_link_pose(base,pyplan.link_from_name(base, 'left_gripper_ee'))) 


# time.sleep(300)
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
    assert(len(joints)==len(jointPoses)) 

    for i,j in enumerate(joints):
        p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i])
        p.stepSimulation()

 
def set_joint_motors(base, jointPoses):
    numJoints = p.getNumJoints(base)
    # for i in range(100):
    for i in range(numJoints):
        jointInfo = p.getJointInfo(base, i)
        qIndex = jointInfo[3]
        if qIndex > -1:
            p.setJointMotorControl2(bodyIndex=base, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[qIndex-7],targetVelocity=0, force=500, positionGain=0.03, velocityGain=1 )
            p.stepSimulation()

def solve_ik(urdf_path, pose):
    urdfstring = ''.join(open(urdf_path,'r').readlines())
    # ik = IK('torso', 'left_gripper_ee',urdf_string=urdfstring)
    ik = IK('panda_link0', 'panda_hand',urdf_string=urdfstring)

    init_joints = [0.0]*7
    (x,y,z),(rx,ry,rz,rw) = pose
    print(x,y,z)
    conf = ik.get_ik(init_joints, x,y,z,rx,ry,rz,rw)
    return conf

def pb_solve_ik(body, pose):
    conf  = p.calculateInverseKinematics(body, 11, pose)
    return conf

def transform_pose(base, world_from_goal):
    #want torso_from_goal
    # tg = tw * wg 
    torso_from_world = invert(get_link_pose(base, 0))
    tg = multiply(torso_from_world, world_from_goal)
    print(tg)
    return tg


def main():
    global base
    pose = ((0.5, 0.5,0.5),(0,0,0,1))
    # conf = solve_ik('buff_digit/panda_arm_hand.urdf', pose)
    # if conf is None:
    #     print('Cannot reach')
    #     return

    conf = pb_solve_ik(base, pose[0])
    # print('len')
    print('conf: ',conf)
    # joints = [10,11,12,13,14,15]
    joints = [0,1,2,3,4,5,6]
    # set_joint_motors(base,conf)
    j=(0.14720490730995048, 0.8472134373227671, 0.8598701236671977, -1.4895870318659121, 2.4598493739297553, 1.1226250704200282, -0.35609815106501336)
    setMotors(base, joints, conf)
    # print('conf: ',conf)
    ee_id = 8
    time.sleep(10)
    print('final pose: ',p.getLinkState(base, ee_id)[0])


def joint_teleop():
    setMotors(base, right_arm_joints,right_rest_conf)
    setMotors(base, left_arm_joints, left_rest_conf)
    while True:
        joint_angles = [x[0] for x in p.getJointStates(base, right_arm_joints)]
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
            print(joint_angles[3])

        if ord('o') in keys:
            joint = right_arm_joints[3]
            angle = p.getJointState(base, joint)[0]
            angle += delta
            control_joint(joint, angle, 0.0, 0.3)
            print(joint_angles[3])

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
        # print(joint_angles)
# time.sleep(1)
        # print(p.getLinkState(base, 10)[0])
# time.sleep(100)

if __name__ == '__main__':
    joint_teleop()
    # transform_pose(base, ((0.3,0,0.7),(0,0,0,1)))
    time.sleep(100)