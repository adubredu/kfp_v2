import numpy as np
import os
import time
import pybullet_data
import pybullet as p
import pybullet_planning as pyplan
from pybullet_planning import Pose, Point
# from trac_ik_python.trac_ik import IK


class Buff_digit:
    def __init__(self, client, base_id):
        self.client = client 
        self.id = base_id
        self.state = "turn"
        self.max_angular_speed = 1.0
        self.max_linear_speed = 2.0
        self.turn_tol = 0.05
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.left_wheel_id = 32
        self.right_wheel_id = 33
        self.wheel_width = 0.8
        self.arms_ee = {'left_arm':16, 
                        'right_arm':31}
        self.arm_joints = {'left_arm':[10,11,12,13,14,15], 
                           'right_arm':[25, 26, 27,28,29,30]}
        self.joint_index_ranges = {'left_arm':(8,14), 
                                   'right_arm':(22,28)}
        self.gripper_joints = {'left_arm':(17,18), 
                               'right_arm':(34,35)}
        self.grasped = {'left_arm':0, 'right_arm':0}
        self.start_up_robot()
        
    def tuck_arms(self):
        self.left_elbow_joint = 13
        self.right_elbow_joint = 28
        self.drive_arm_joints([self.left_elbow_joint,
            self.right_elbow_joint], [-1.0,1.0])


    def start_up_robot(self):
        self.lowerLimits,self.upperLimits,self.jointRanges,self.restPoses = self.get_joint_ranges()
        # self.open_gripper('left_arm')
        # self.open_gripper('right_arm')

    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff < -np.pi: diff += 2.0*np.pi
        while diff > np.pi: diff -= 2.0*np.pi
        return diff 


    def follow_path(self, path):
        length = len(path)
        for i,pose in enumerate(path):
            self.move_to_pose(pose, last=(i==length-1))
        print('at goal pose')


    def plan_and_drive_to_pose(self, goal_pose, limits,obstacles=[]):
        # time.sleep(3)
        # self.tuck_arms()
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        self.follow_path(path)
        # self.drive_along_path(path)


    def drive_along_path(self, path):
        for pose in path:
            pyplan.set_pose(self.id, ((pose[0],pose[1],0.3), p.getQuaternionFromEuler((0,0,pose[2]))))
            time.sleep(0.05)

    
    def drive_base(self, linear_speed, angular_speed):
        left_vel = linear_speed - angular_speed*(self.wheel_width*0.5)
        right_vel = linear_speed + angular_speed*(self.wheel_width*0.5)
        # p.setJointMotorControl2(self.id, self.left_wheel1_id, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
        # p.setJointMotorControl2(self.id, self.left_wheel2_id, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
        # p.setJointMotorControl2(self.id, self.right_wheel1_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)
        # p.setJointMotorControl2(self.id, self.right_wheel2_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)

        p.setJointMotorControl2(self.id, self.left_wheel_id, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
        p.setJointMotorControl2(self.id, self.right_wheel_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)


    def move_to_pose(self, pose, last=False):
        orientation = pose[2]
        self.state = "turn"
        while not (self.state == "done"):
            current_pose = p.getBasePositionAndOrientation(self.id, self.client)[0]
            current_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id, self.client)[1])[2]
            target_heading = np.arctan2(pose[1] - current_pose[1], pose[0]-current_pose[0])
            error = self.angle_diff(target_heading, current_yaw)
            if orientation > 0.01:
                final_yaw_error = self.angle_diff(orientation, current_yaw)
            angular_speed = 0.0; linear_speed = 0.0
            # print(error)
            if self.state == "turn":
                if np.abs(error) > self.turn_tol:
                    linear_speed = 0.0
                    angular_speed = 0.0
                    if error > 0:
                        angular_speed = self.max_angular_speed
                    else:
                        angular_speed = -self.max_angular_speed

                else:
                    linear_speed = self.max_linear_speed
                    self.state = "drive"

            elif self.state == "drive":
                self.tuck_arms()
                linear_speed = self.max_linear_speed
                dist_to_goal = np.linalg.norm(np.array(pose[:2])-np.array(current_pose[:2]))

                tol = self.trans_tol
                if last: tol = self.final_trans_tol
                
                if dist_to_goal < self.trans_tol:                   
                    linear_speed = 0.0
                    if orientation > 0.01:
                        self.state = "there"
                    else:
                        self.state = "done"
                        if not last:
                            linear_speed=self.max_linear_speed

            elif self.state == "there":
                if np.abs(final_yaw_error) > self.turn_tol:
                    linear_speed = 0.0
                    if final_yaw_error > 0.0:
                        angular_speed = self.max_angular_speed
                    else:
                        angular_speed = -self.max_angular_speed
                else:
                    self.state = "done"
            # print('Current position: %.2f %.2f %.2f'%(current_pose[0],current_pose[1],current_yaw))
            # print(self.state)
            self.drive_base(linear_speed, angular_speed)
        return True


    def get_joint_ranges(self, includeFixed=False):
        all_joints = pyplan.get_movable_joints(self.id)
        upper_limits = pyplan.get_max_limits(self.id, all_joints)
        lower_limits = pyplan.get_min_limits(self.id, all_joints)

        jointRanges, restPoses = [], []

        numJoints = p.getNumJoints(self.id)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)

            if includeFixed or jointInfo[3] > -1:
                rp = p.getJointState(self.id, i)[0]
                jointRanges.append(2)
                restPoses.append(rp)

        return lower_limits, upper_limits, jointRanges, restPoses


    def solve_ik(self, position, orientation, ee_id):
        conf = p.calculateInverseKinematics(self.id, ee_id, position,orientation, lowerLimits=self.lowerLimits, upperLimits=self.upperLimits, jointRanges=self.jointRanges, maxNumIterations=100000,residualThreshold=0.001, restPoses=self.restPoses)
        return conf


    def drive_arm_joints(self, joints, jointPoses):
        assert(len(joints)==len(jointPoses))

        for i,j in enumerate(joints):
            p.setJointMotorControl2(bodyIndex=self.id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i], maxVelocity=0.5)#, positionGain=0.001)
            p.stepSimulation()


    def plan_and_execute_arm_motion(self, position, orientation, armname):
        ee_id = self.arms_ee[armname]
        conf = self.solve_ik(position,orientation,ee_id)
        joints = self.arm_joints[armname]
        ranges = self.joint_index_ranges[armname]
        joint_confs = conf[ranges[0]:ranges[1]]
        self.drive_arm_joints(joints, joint_confs)


    def open_gripper(self, armname):
        gripper_joints = self.gripper_joints[armname]
        conf = [0.025,0.025]
        self.drive_arm_joints(gripper_joints,conf)


    def close_gripper(self, armname):
        gripper_joints = self.gripper_joints[armname]
        conf = [0.0,0.0]
        self.drive_arm_joints(gripper_joints,conf)


    def get_top_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[2]+=(height/2.0)
        orientation = p.getQuaternionFromEuler((1.57,1.57,0))
        return position, orientation 

    def get_side_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        width = aabb[1][0] - aabb[0][0]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[0] -=(width/2.0)
        orientation = p.getQuaternionFromEuler((1.57,0,0))
        return position, orientation 

    def hold(self, object_id, armname):
        ee_id = self.arms_ee[armname]
        self.grasped[armname] = p.createConstraint(self.id, ee_id,object_id,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[0,0,0])

    def release_hold(self, armname):
        p.removeConstraint(self.grasped[armname])

        


















if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models')
    kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
    with pyplan.HideOutput(enable=True):
        floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
        kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        table1 = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=False)
        table2 = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
        # pepsi = p.loadSDF('can_pepsi/model.sdf')
        # pepsi = pepsi[0]
        # pyplan.set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=pyplan.stable_z(pepsi, table1))))
        robot_id = p.loadURDF('buff_digit/digit_freight.urdf',[0,0,0])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        pepsi = p.loadURDF("cube_small.urdf")
        pyplan.set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=pyplan.stable_z(pepsi, table1))))

        
    robot = Buff_digit(client, robot_id)
    # for i in range(10):
    #     robot.plan_and_drive_to_pose([-4,0,0.0],[100,100])
    #     time.sleep(2)
    #     robot.plan_and_drive_to_pose([0,0,0.0],[100,100])
    #     time.sleep(2)
    # print(pyplan.get_movable_joints(robot.robot))
    
    grasp_position, grasp_orientation = robot.get_side_grasp(pepsi)
    robot.plan_and_execute_arm_motion(grasp_position, grasp_orientation,'right_arm')
    time.sleep(7)
    # robot.tuck_arms()
    robot.hold(pepsi, 'right_arm')
    time.sleep(3)
    robot.plan_and_drive_to_pose([-3,0,0.0],[100,100])
    robot.plan_and_execute_arm_motion(grasp_position,grasp_orientation,'right_arm')
    time.sleep(8)
    robot.release_hold('right_arm')
    time.sleep(200)
    