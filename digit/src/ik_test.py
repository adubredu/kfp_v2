import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
from pybullet_planning import get_movable_joints, plan_joint_motion


clid = p.connect(p.SHARED_MEMORY)


if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath('/home/bill/garage/kfp_v2/digit/models')
p.loadURDF('floor/floor.urdf')
kukaId = p.loadURDF("a1/a1.urdf", useFixedBase=True)


kukaEndEffectorIndex = 15
all_joints = [10,11,12,13,14,15]
numJoints = 6

def getJointRanges(robotId, all_joints, includeFixed=False):
        lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []

        for i in all_joints:
            jointInfo = p.getJointInfo(kukaId, i)
            if includeFixed or jointInfo[3] > -1:
                ll, ul = jointInfo[8:10]
                jr = ul - ll
                # For simplicity, assume resting state == initial state
                rp = p.getJointState(robotId, i)[0]

                lowerLimits.append(-2)
                upperLimits.append(2)
                jointRanges.append(2)
                restPoses.append(rp)
        return lowerLimits, upperLimits, jointRanges, restPoses


ll,ul,jr,rp = getJointRanges(kukaId, all_joints)

for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])

p.setGravity(0, 0, -10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0
trailDuration = 15

useOrientation = 0
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 0
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)






while 1:
  p.stepSimulation()

  for i in range(1):
    #pos = [-0.4,0.2*math.cos(t),0.+0.2*math.sin(t)]
    pos = [0.2 * math.cos(t), 0, 0. + 0.2 * math.sin(t) + 0.2]
    #end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul,jr, rp)

    goals = jointPoses[8:15]
    # print(goals)
    t = t + 0.001
    for i in range(numJoints):
      p.setJointMotorControl2(bodyIndex=kukaId,
                              jointIndex=i+9,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=goals[i],
                              targetVelocity=0,
                              force=500,
                              positionGain=1,
                              velocityGain=0.1)
    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    if (hasPrevPose):
      p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
      p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1


  