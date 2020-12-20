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
from pybullet_planning import plan_base_motion

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
        self.client = 0
        self.state = "turn"
        self.max_angular_speed = 0.25
        self.max_linear_speed = 0.3
        self.turn_tol = 0.5
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.wheel_width = 0.2

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


    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff < -np.pi: diff += 2.0*np.pi
        while diff > np.pi: diff -= 2.0*np.pi
        return diff 


    def follow_path(self, path):
        length = len(path)
        for i,pose in enumerate(path):
            self.move_to_pose(pose, last=(i==length-1))


    def plan_and_drive_to_pose(self, goal_pose, limits,obstacles=[]):
        path = plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        self.follow_path(path)

    ###
    def drive_base(self, linear_speed, angular_speed):
        # print('lin ang: ',linear_speed, angular_speed)
        self._update_controller_params(linear_speed, angular_speed)
        self.controller.update()
        hybrid_action,_ = self.controller.get_action()
        # print(hybrid_action)
        self.robot.Step(hybrid_action)
        # time.sleep(0.3)


    def move_to_pose(self, pose, last=False):
        print('moving to: ',pose)
        orientation = pose[2]
        self.state = "turn"
        while not (self.state == "done"):
            current_pose = self.robot.GetBasePosition()
            current_yaw = self.robot.GetTrueBaseRollPitchYaw()[2]
            target_heading = np.arctan2(pose[1] - current_pose[1], pose[0]-current_pose[0])
            error = self.angle_diff(target_heading, current_yaw)
            print('error: ',error)
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

if __name__ == '__main__':
    # client = pybullet.connect(pybullet.GUI);print(client)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    # p.setRealTimeSimulation(1)
    p.setTimeStep(0.001)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models')
    p.loadURDF('floor/floor.urdf')
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    base = Base(p)
    base.plan_and_drive_to_pose([5,5,0.0],[100,100])
    time.sleep(10)



