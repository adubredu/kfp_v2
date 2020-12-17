import pybullet as p 
import pybullet_data
import numpy as np
import time
from pybullet_planning import get_movable_joints, plan_joint_motion


class Arm:
    def __init__(self, client, robot_id, ee_id):
        self.client = client
        self.robot_id = robot_id
        self.ee_id = ee_id
        self.movable_joints= get_movable_joints(robot_id)
        self.lowerLimits, self.upperLimits, self.jointRanges, self.restPoses = self.getJointRanges()
        self.neutral_joint_confs = [-0.0015158812087194544, -0.0006311930743239932, -0.8437690555059353, -0.0026768981080225146, -0.011999084629964712, -0.00031838324849879553, -0.04278477797888542, 0.004207259121201607, 0.03368973046943163, 0.0004851727128079414, 1.6945922805840093e-06, 0.8910043245567677, -0.001332861078622622, 0.0005842182629561161, 0.0023770515534108296, 0.035435049141974345, -0.0053353071553770386, -0.03482688305331531, 0.00019865396349139517, -7.66858844373079e-10]
        self.setMotors(self.neutral_joint_confs, self.movable_joints)
        self.right_joints = [9,10,11,12,13,14,15]
        time.sleep(2)



    def getJointRanges(self, includeFixed=False):
        """
        Parameters
        ----------
        self.robot_id : int
        includeFixed : bool
        Returns
        -------
        lowerLimits : [ float ] * numDofs
        upperLimits : [ float ] * numDofs
        jointRanges : [ float ] * numDofs
        restPoses : [ float ] * numDofs
        """

        lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []

        for i in self.movable_joints:
            jointInfo = p.getJointInfo(self.robot_id, i)

            if includeFixed or jointInfo[3] > -1:

                ll, ul = jointInfo[8:10]
                jr = ul - ll

                # For simplicity, assume resting state == initial state
                rp = p.getJointState(self.robot_id, i)[0]

                lowerLimits.append(-2)
                upperLimits.append(2)
                jointRanges.append(2)
                restPoses.append(rp)


        return lowerLimits, upperLimits, jointRanges, restPoses


    def accurateIK(self, targetPosition, targetOrientation, useNullSpace=True, maxIter=10):
        """
        Parameters
        ----------
        self.robot_id : int
        endEffectorId : int
        targetPosition : [float, float, float]
        targetOrientation : quaternion
        lowerLimits : [float] 
        upperLimits : [float] 
        jointRanges : [float] 
        restPoses : [float]
        useNullSpace : bool
        maxIter : int
        threshold : float
        Returns
        -------
        jointPoses : [float] * numDofs
        """
        iter = 0

        while (iter<maxIter):
            if useNullSpace:
                jointPoses = p.calculateInverseKinematics(self.robot_id, self.ee_id, targetPosition, targetOrientation, lowerLimits=self.lowerLimits, upperLimits=self.upperLimits, jointRanges=self.jointRanges,restPoses=self.restPoses)
            else:
                jointPoses = p.calculateInverseKinematics(self.robot_id, self.ee_id, targetPosition, targetOrientation)
            iter +=1
        print('num of joints from ik: ',len(jointPoses))
        return jointPoses
    

    def setMotors(self, jointPoses, joints):
        """
        Parameters
        ----------
        self.robot_id : int
        jointPoses : [float] * numDofs
        """
        if len(jointPoses) != len(joints):
            print('wrong number of joints')
            return

        ind = 0    
        for i in joints:
            jointInfo = p.getJointInfo(self.robot_id, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[ind],targetVelocity=0, force=500, positionGain=0.03, velocityGain=1 )
            ind += 1


    def plan_and_execute_path_to_pose(self, position, orientation, obstacles):
        goal_joints = self.accurateIK(position, orientation)
        g = goal_joints[2:9]
        g= (-0.9616992060859841, 0.7, 0.241440948750928, -1.983222313726789, -0.10431123833445043, -1.07, 0.5584585935730565)
        # print(g)
        # self.setMotors([np.pi,-np.pi/2,np.pi/2,0,0,0,0], self.right_joints)
        # print('target joints:',self.right_joints)
        path = plan_joint_motion(self.robot_id, self.right_joints, g, obstacles=obstacles, self_collisions=False)
        if path is not None:
            # print(path)
            for conf in path:
                self.setMotors(conf, self.right_joints)
                time.sleep(0.03)
        else:
            # print(goal_joints)
            # self.setMotors(goal_joints)
            print('no path found')



