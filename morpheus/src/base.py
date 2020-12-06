import pybullet as p 
import pybullet_data
import numpy as np
import time


class base:
	def __init__(self, client, base_id):
		self.client = client 
		self.id = base_id
		self.state = "turn"
		self.max_angular_speed = 2.5
		self.max_linear_speed = 5.0
		self.turn_tol = 0.005
		self.trans_tol = 0.05
		self.left_wheel_id = 0
		self.right_wheel_id = 1
		self.wheel_width = 0.5

	def angle_diff(self, angle1, angle2):
		diff = angle2 - angle1
		while diff < -np.pi: diff += 2.0*np.pi
		while diff > np.pi: diff -= 2.0*np.pi
		return diff 

	def drive_base(self, linear_speed, angular_speed):
		left_vel = linear_speed - angular_speed*(self.wheel_width*0.5)
		right_vel = linear_speed + angular_speed*(self.wheel_width*0.5)
		p.setJointMotorControl2(self.id, self.left_wheel_id, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
		p.setJointMotorControl2(self.id, self.right_wheel_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)

	def move_to_pose(self, pose, orientation):
		self.state = "turn"
		while not (self.state == "done"):
			current_pose = p.getBasePositionAndOrientation(self.id, self.client)[0]
			current_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id, self.client)[1])[2]
			target_heading = np.arctan2(pose[1] - current_pose[1], pose[0]-current_pose[0])
			error = self.angle_diff(target_heading, current_yaw)
			if orientation is not None:
				final_yaw_error = self.angle_diff(orientation[2], current_yaw)
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
					self.state = "drive"

			elif self.state == "drive":
				linear_speed = self.max_linear_speed
				dist_to_goal = np.linalg.norm(np.array(pose[:2])-np.array(current_pose[:2]))

				if dist_to_goal < self.trans_tol:
					linear_speed = 0.0
					if orientation is not None:
						self.state = "there"
					else:
						self.state = "done"

			elif self.state == "there":
				if np.abs(final_yaw_error) > self.turn_tol:
					linear_speed = 0.0
					if final_yaw_error > 0.0:
						angular_speed = self.max_angular_speed
					else:
						angular_speed = -self.max_angular_speed
				else:
					self.state = "done"
			print('Current position: %.2f %.2f %.2f'%(current_pose[0],current_pose[1],current_yaw))
			self.drive_base(linear_speed, angular_speed)
		return True





if __name__ == '__main__':
	client = p.connect(p.GUI)  
	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
	p.setRealTimeSimulation(1)
	p.setGravity(0, 0, -9.81)

	p.setAdditionalSearchPath('/home/bill/kfp/kfp_v2/morpheus/models')
	p.loadURDF('floor/floor.urdf')
	base_id = p.loadURDF('fetch_description/robots/freight.urdf')
	base = base(client, base_id)
	base.move_to_pose([2.0,0.0,0],None)
	base.move_to_pose([2.0,2.0,0],None)
	base.move_to_pose([0.0,2.0,0],None)
	base.move_to_pose([0.0,0.0,0],[0.0, 0.0, 0.0])
	print('here')
	time.sleep(100)


		