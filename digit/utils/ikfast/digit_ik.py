import numpy as np 
import sys
import os 
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),'left_arm'))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),'right_arm'))
from .left_arm.left_arm_ik import left_ik
from .right_arm.right_arm_ik import right_ik
import pybullet_planning as pyplan 

def solve_ik(position, orientation, armname):
	position = list(position)
	if armname == 'left_arm':
		ik = left_ik
	else:
		ik = right_ik
		position[1]+=0.15
	n_joints = ik.getDOF()
	tf_matrix = pyplan.tform_from_pose((position, orientation))
	ee_pose = tf_matrix[:-1]
	joint_confs = ik.inverse(ee_pose.reshape(-1).tolist())
	n_solutions = int(len(joint_confs)/n_joints) 
	joint_confs = np.asarray(joint_confs).reshape(n_solutions,n_joints)
	if len(joint_confs) == 0: 
		yield None
	for conf in joint_confs: 
		yield conf 

if __name__=='__main__':
	gen = solve_ik((0,0,0), (0,0,0,1),'right_arm')
	a = next(gen)
	print(a)
