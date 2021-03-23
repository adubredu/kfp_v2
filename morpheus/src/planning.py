import pybullet_planning as pp
import pybullet as p 
import pybullet_data 
import numpy as np
from pybullet_planning.interfaces.env_manager import Point, Pose, Euler
from pybullet_planning.interfaces.planner_interface.cartesian_motion_planning import plan_cartesian_motion
from pybullet_planning.interfaces.planner_interface.SE2_pose_motion_planning import plan_base_motion
from pybullet_planning.interfaces.planner_interface.joint_motion_planning import get_difference_fn, get_sample_fn, get_extend_fn
from pybullet_planning.interfaces.robots.collision import get_collision_fn
from pybullet_planning import plan_joint_motion, set_joint_positions, get_movable_joints
import time 
from base import Base
from arm import Arm

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath('../models')
p.loadURDF('floor/floor.urdf')
# dof = p.loadURDF('dof/dof.urdf')
mbase = p.loadURDF('morpheus_description/morpheus.urdf')
# start_loc = Point(0.0,0.0,0.0)
# end_loc = Point(5,5,0)
# orientation = Euler(0.0,0.0,0.0)
# start_pose = Pose(start_loc, orientation)
# end_pose = Pose(end_loc, orientation)

# # path = plan_cartesian_motion(dof, 0, 11, [end_pose])
# time.sleep(3)
# path = plan_base_motion(mbase, [5.0,5.0,0.0], [100,100])
# print('path: ',path)
# print('final pose: ',p.getLinkState(dof, 11)[0])
# print('final joint angles: ',[x[0] for x in p.getJointStates(dof, [0,2,4,6,8,10])])
print(get_movable_joints(mbase))

# base = Base(client, mbase)
# base.plan_and_drive_to_pose([.5,.5,0.0],[100,100])
# time.sleep(3)
arm = Arm(client, mbase, 16)
time.sleep(1)
arm.plan_and_execute_path_to_pose([0.5061, -0.2507, 0.5829], [0,0,0,1], obstacles=[])
print('final pose: ',p.getLinkState(mbase, 16)[0])
# print(p.inverseKinematics(mbase, 13,[0.5,0.1,0.7] ))
# left_arm_joints = [19,20,21,22,23,24]
# final_point = np.array([0.5,0.1,0.7])
# euler = np.zeros(3)
# final_conf = np.concatenate([final_point, euler])
#use inverse kinematics to figure out the required final joint values
#then feed them into this function below


time.sleep(10)
joint_angles = [x[0] for x in p.getJointStates(mbase, get_movable_joints(mbase))]
print(joint_angles)
      # print(joint_angles)
#obstacles[] are body ids