import pybullet_planning as pp
import pybullet as p 
import pybullet_data 
import numpy as np
from pybullet_planning.interfaces.env_manager import Point, Pose, Euler
from pybullet_planning.interfaces.planner_interface.cartesian_motion_planning import plan_cartesian_motion
from pybullet_planning.interfaces.planner_interface.SE2_pose_motion_planning import plan_base_motion
from pybullet_planning.interfaces.planner_interface.joint_motion_planning import get_difference_fn, get_sample_fn, get_extend_fn
from pybullet_planning.interfaces.robots.collision import get_collision_fn
import time 
from base import Base

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath('../models')
p.loadURDF('floor/floor.urdf')
# dof = p.loadURDF('dof/dof.urdf')
mbase = p.loadURDF('fetch_description/robots/freight.urdf')
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
base = Base(client, mbase)
base.plan_and_drive_to_pose([5.0,5.0,0.0],[100,100])

time.sleep(100)

#obstacles[] are body ids