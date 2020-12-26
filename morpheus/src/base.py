import pybullet as p 
import pybullet_data
import numpy as np
import time
from pybullet_planning.interfaces.env_manager.pose_transformation import base_values_from_pose, get_pose
from pybullet_planning.interfaces.planner_interface.SE2_pose_motion_planning import plan_base_motion


class Base:
	def __init__(self, client, base_id):
		self.client = client 
		self.id = base_id
		self.state = "turn"
		self.max_angular_speed = 2.5
		self.max_linear_speed = 5.0
		self.turn_tol = 0.05
		self.trans_tol = 0.3
		self.final_trans_tol = 0.05
		self.left_wheel_id = 36
		self.right_wheel_id = 37
		self.wheel_width = 0.5

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

	def drive_base(self, linear_speed, angular_speed):
		left_vel = linear_speed - angular_speed*(self.wheel_width*0.5)
		right_vel = linear_speed + angular_speed*(self.wheel_width*0.5)
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
			self.drive_base(linear_speed, angular_speed)
		return True










if __name__ == '__main__':
	client = p.connect(p.GUI)  
	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
	p.setRealTimeSimulation(1)
	p.setGravity(0, 0, -9.81)

	p.setAdditionalSearchPath('/home/bill/garage/kfp_v2/')
	p.loadURDF('morpheus/models/floor/floor.urdf')
	# base_id = p.loadURDF('morpheus/models/fetch_description/robots/freight.urdf')

	base_id = p.loadURDF('digit/models/buff_digit/digit_freight.urdf')
	base = Base(client, base_id)
	# base.move_to_pose([2.0,0.0,None])
	# base.move_to_pose([2.0,2.0,None])
	# base.move_to_pose([0.0,2.0,None])
	# base.move_to_pose([0.0,0.0, 0.0])
	p.setJointMotorControl2(base_id, 11,
				controlMode=p.POSITION_CONTROL,targetPosition=1.57,
				force=3000)
	p.setJointMotorControl2(base_id, 28,
				controlMode=p.POSITION_CONTROL,targetPosition=-1.57,
				force=3000)
	base.plan_and_drive_to_pose([5.0,5.0,0.0],[100,100])
	print('here')
	time.sleep(100)


		