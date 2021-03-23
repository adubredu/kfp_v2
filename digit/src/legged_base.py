import numpy as np
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from absl import app
from absl import flags
from absl import logging
from datetime import datetime
import scipy.interpolate
import time
import pybullet_data
from pybullet_utils import bullet_client
import pybullet 
from pybullet_planning import plan_base_motion, plan_joint_motion, set_joint_positions, get_movable_joints, get_max_limits, get_min_limits,set_pose, get_pose, Pose, Point, stable_z
import pybullet_planning as ut

from mpc_controller import com_velocity_estimator
from mpc_controller import gait_generator as gait_generator_lib
from mpc_controller import locomotion_controller
from mpc_controller import openloop_gait_generator
from mpc_controller import raibert_swing_leg_controller
#from mpc_controller import torque_stance_leg_controller
from mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller
from motion_imitation.robots import a1, robot_config
p = bullet_client.BulletClient(connection_mode=pybullet.GUI)

class Base:
    def __init__(self,p):
        self.robot = a1.A1(p,
                  motor_control_mode=robot_config.MotorControlMode.HYBRID,
                  enable_action_interpolation=False,
                  reset_time=2,
                  time_step=0.002,
                  action_repeat=1)
        self.id = self.robot.quadruped
        self.p = p
        self.client = 0
        self.state = "turn"
        self.max_angular_speed = 0.25
        self.max_linear_speed = 0.3
        self.turn_tol = 0.5
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.wheel_width = 0.2
        self.left_ee = 9
        self.right_ee = 17
        self.arm_ee={'left_arm': self.left_ee, 'right_arm': self.right_ee}
        self.arm_ranges={'left_arm':(0,6), 'right_arm':(8,14)}

        self.arm_joints = [2,3,4,5,6,7,8,9,11,12,13,14,15,16,17,18]
        self.left_arm_joints = [2,3,4,5,6,7]
        self.right_arm_joints = [10,11,12,13,14,15]
        self.arm_joints_dict = {'left_arm':self.left_arm_joints, 'right_arm':self.right_arm_joints}
        self.gripper_joints = {'left_arm':(8,9), 'right_arm':(16,17)}
        self.lowerLimits, self.upperLimits, self.jointRanges, self.restPoses = self.getJointRanges()
        # self.brace_arms()
        self.controller = self._setup_controller(self.robot)
        self.controller.reset()


    def _setup_controller(self, robot):
        desired_speed = (0, 0)
        desired_twisting_speed = 0
        NUM_SIMULATION_ITERATION_STEPS = 300
        _MAX_TIME_SECONDS = 30.
        _STANCE_DURATION_SECONDS = [0.3] * 4
        _DUTY_FACTOR = [0.6] * 4
        _INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]

        _INIT_LEG_STATE = (
            gait_generator_lib.LegState.SWING,
            gait_generator_lib.LegState.STANCE,
            gait_generator_lib.LegState.STANCE,
            gait_generator_lib.LegState.SWING,
        ) 

        gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
          robot,
          stance_duration=_STANCE_DURATION_SECONDS,
          duty_factor=_DUTY_FACTOR,
          initial_leg_phase=_INIT_PHASE_FULL_CYCLE,
          initial_leg_state=_INIT_LEG_STATE)
        window_size = 20 
        state_estimator = com_velocity_estimator.COMVelocityEstimator(
          robot, window_size=window_size)
        sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
          robot,
          gait_generator,
          state_estimator,
          desired_speed=desired_speed,
          desired_twisting_speed=desired_twisting_speed,
          desired_height=robot.MPC_BODY_HEIGHT,
          foot_clearance=0.01)

        st_controller = torque_stance_leg_controller.TorqueStanceLegController(
          robot,
          gait_generator,
          state_estimator,
          desired_speed=desired_speed,
          desired_twisting_speed=desired_twisting_speed,
          desired_body_height=robot.MPC_BODY_HEIGHT)

        controller = locomotion_controller.LocomotionController(
          robot=robot,
          gait_generator=gait_generator,
          state_estimator=state_estimator,
          swing_leg_controller=sw_controller,
          stance_leg_controller=st_controller,
          clock=robot.GetTimeSinceReset)
        return controller

    def _update_controller_params(self, lin_speed, ang_speed):
        self.controller.swing_leg_controller.desired_speed = lin_speed
        self.controller.swing_leg_controller.desired_twisting_speed = ang_speed
        self.controller.stance_leg_controller.desired_speed = lin_speed
        self.controller.stance_leg_controller.desired_twisting_speed = ang_speed


    def brace_arms(self):
        for joint in self.left_arm_joints+self.right_arm_joints:
            p.setJointMotorControl2(self.id, joint,
                controlMode=p.POSITION_CONTROL,targetPosition=0.0,
                force=3000)


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
        path = plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        self.follow_path(path)

    
    def drive_base(self, linear_speed, angular_speed):
        self.brace_arms()
        self._update_controller_params(linear_speed, angular_speed)
        self.controller.update()
        hybrid_action,_ = self.controller.get_action()
        self.robot.Step(hybrid_action)


    def move_to_pose(self, pose, last=False):
        orientation = pose[2]
        self.state = "turn"
        while not (self.state == "done"):
            current_pose = self.robot.GetBasePosition()
            current_yaw = self.robot.GetTrueBaseRollPitchYaw()[2]
            target_heading = np.arctan2(pose[1] - current_pose[1], pose[0]-current_pose[0])
            error = self.angle_diff(target_heading, current_yaw)
            if orientation > 0.01:
                final_yaw_error = self.angle_diff(orientation, current_yaw)
            angular_speed = 0.0; linear_speed = 0.0

            if self.state == "turn":
                if np.abs(error) > self.turn_tol:
                    linear_speed = 0.0
                    angular_speed = 0.0
                    if error > 0:
                        angular_speed = -self.max_angular_speed
                    else:
                        angular_speed = self.max_angular_speed

                else:
                    linear_speed = self.max_linear_speed
                    self.state = "drive"

            elif self.state == "drive":
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
            lspeed = (linear_speed, 0.0, 0.0)
            self.drive_base(lspeed, angular_speed)
        return True


    def getJointRanges(self, includeFixed=False):
        all_joints = get_movable_joints(self.id)
        upper_limits = get_max_limits(self.id, all_joints)
        lower_limits = get_min_limits(self.id, all_joints)

        jointRanges, restPoses = [], []

        numJoints = p.getNumJoints(self.id)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)

            if includeFixed or jointInfo[3] > -1:
                rp = p.getJointState(self.id, i)[0]
                jointRanges.append(2)
                restPoses.append(rp)

        return lower_limits, upper_limits, jointRanges, restPoses


    def solve_ik(self, targetPosition, targetOrientation, ee_id, useNullSpace=True):
        if useNullSpace:
            jointPoses = p.calculateInverseKinematics(self.id, ee_id, targetPosition, targetOrientation,
                lowerLimits=self.lowerLimits, upperLimits=self.upperLimits, jointRanges=self.jointRanges, maxNumIterations=100000,residualThreshold=0.001, restPoses=self.restPoses)
        else:
            jointPoses = p.calculateInverseKinematics(self.id, ee_id, targetPosition)
        return jointPoses

    def control_joint(self, joint, value):
        p.setJointMotorControl2(self.id, joint,
                    controlMode=p.POSITION_CONTROL,targetPosition=value,targetVelocity=10,force=3000)
        # for i in range(10):
        p.stepSimulation()

    def open_gripper(self, arm_name):
        j = self.gripper_joints[arm_name]
        self.control_joint(j[0], 0.025)
        self.control_joint(j[1], 0.025)
        print('gripper open')


    def close_gripper(self, arm_name):
        j = self.gripper_joints[arm_name]
        self.control_joint(j[0], 0.0)
        self.control_joint(j[1], 0.0) 
        print('gripper closed')


    def set_joint_motors(self, jointPoses):
        numJoints = p.getNumJoints(self.id)
        # for i in range(100):
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.setJointMotorControl2(bodyIndex=self.id, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[qIndex-7],targetVelocity=0, force=500, positionGain=0.03, velocityGain=1 )
                p.stepSimulation()
                # print('i')

    def move_arm_to_pose_ik(self, position, orientation, arm_name):
        ee_id = self.arm_ee[arm_name]
        for i in range(20):
            joint_configs = self.solve_ik(position, orientation, ee_id)
            self.set_joint_motors(joint_configs)

        # js = joint_configs[self.arm_ranges[arm_name][0]:self.arm_ranges[arm_name][1]]
        # set_joint_positions(self.id, self.arm_joints_dict[arm_name], js)
        # print(joint_configs)
        # print('there')
        print('final pose: ',p.getLinkState(self.id, ee_id)[0])
        print('final orientation: ',p.getEulerFromQuaternion(p.getLinkState(self.id, ee_id)[1]))


    def plan_and_execute_arm_in_jointspace(self, position, orientation, arm_name):
        arm_joints = self.arm_joints_dict[arm_name]
        ee_id = self.arm_ee[arm_name]
        goal_joints = self.solve_ik(position, orientation, ee_id)
        arm_goal_joints = goal_joints[self.arm_ranges[arm_name][0]:self.arm_ranges[arm_name][1]]
        self.brace_arms()
        print('braced')
        time.sleep(4)
        path = plan_joint_motion(self.id, arm_joints, arm_goal_joints)
        print(path)




if __name__ == '__main__':
    # client = pybullet.connect(pybullet.GUI);print(client)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    # p.setRealTimeSimulation(1)
    p.setTimeStep(0.001)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models')
    # p.loadURDF('floor/floor.urdf')
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())

    kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
    with ut.HideOutput(enable=True):
        floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
        kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)

    # z = ut.stable_z(kitchen, floor) - ut.get_point(floor)[2]
    # point = np.array(ut.get_point(kitchen)) - np.array([0, 0, z])
    # ut.set_point(floor, point)


    table1 = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=False)
    table2 = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
    pepsi = p.loadSDF('can_pepsi/model.sdf')
    pepsi = pepsi[0]
    # block = p.loadURDF('block/block.urdf')
    set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=stable_z(pepsi, table1))))
    block_pose = get_pose(pepsi)
    base = Base(p)
    # for i in range(p.getNumJoints(base.id)):
    #     print('')
    #     print(p.getJointInfo(base.id, i))
    # not_digit = p.loadURDF('not_digit/nl_digit.urdf',[0,0,0.4])

    # cid = p.createConstraint(base.id, -1, not_digit, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0], p.getQuaternionFromEuler([0, 1.5707, 0]))
    time.sleep(5)
    # base.plan_and_drive_to_pose([-4,4,0.0],[100,100])
    time.sleep(2)
    base.plan_and_drive_to_pose([-4,0,0.0],[100,100])
    position = [0.6,0.2,0.8]; orientation=p.getQuaternionFromEuler((0,1,0))
    time.sleep(3); print(block_pose[0],(0,1,0))
    # base.move_arm_to_pose_ik(block_pose[0],orientation,'right_arm')
    # base.close_gripper('right_arm')
    time.sleep(200)




