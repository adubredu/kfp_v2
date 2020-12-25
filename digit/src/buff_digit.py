import numpy as np
import os
import time
import pybullet_data
import pybullet as p
import pybullet_planning as pyplan
from pybullet_planning import Pose, Point
# from trac_ik_python.trac_ik import IK


class Buff_digit:
    def __init__(self, pose=[0,0,0],orientation=[0,0,0,1]):
        robot_path = 'buff_digit/buff_digit.urdf'
        with pyplan.HideOutput(enable=True):
            self.robot = p.loadURDF(robot_path, pose, orientation)
        self.arm_ee={'left_arm': 13, 'right_arm': 26}
        self.arm_base_name={'left_arm': 'left_panda_link0', 'right_arm': 'right_panda_link0'}
        self.arm_tool_link={'left_arm': 'left_tool_link', 'right_arm': 'right_tool_link'}
        self.arm_tip_link={'left_arm': 'left_panda_link8', 'right_arm': 'right_panda_link8'}
        self.arm_joint_ranges={'left_arm':(2,9), 'right_arm':(15,22)} 
        self.arm_grippers={'left_arm':(11,12), 'right_arm':(24,25)} 
        self.arm_ik_solvers = {}
        panda_urdf_path = '../models/buff_digit/panda_arm_hand.urdf'
        # self.initialize_ik(panda_urdf_path,'left_arm')
        # self.initialize_ik(panda_urdf_path,'right_arm')



    def initialize_ik(self, urdf_path, arm_name):
        base_link = self.arm_base_name[arm_name]
        tip_link = self.arm_tip_link[arm_name]
        solver = IK(base_link=str(base_link), tip_link=str(tip_link),
            timeout=0.01, epsilon=1e-5, solve_type="Speed",urdf_string=pyplan.read(urdf_path))
        self.arm_ik_solvers[arm_name] = solver 


    def solve_ik(self, world_from_tool, arm_name, nearby_tolerance=np.inf):
        solver = self.arm_ik_solvers[arm_name]
        init_lower, init_upper = solver.get_joint_limits()
        base_link = pyplan.link_from_name(self.robot, solver.base_link)
        world_from_base = pyplan.get_link_pose(self.robot, base_link)
        tip_link = pyplan.link_from_name(self.robot, solver.tip_link)
        tool_from_tip = pyplan.multiply(pyplan.invert(pyplan.get_link_pose(self.robot, self.arm_ee[arm_name])), pyplan.get_link_pose(self.robot, tip_link))
        world_from_tip = pyplan.multiply(world_from_tool, tool_from_tip)
        base_from_tip = pyplan.multiply(invert(world_from_base), world_from_tip)
        joints = pyplan.joints_from_names(self.robot, solver.joint_names)
        seed_state = pyplan.get_joint_positions(self.robot, joints)

        lower, upper = init_lower, init_upper
        if nearby_tolerance < np.inf:
            tolerance = nearby_tolerance*np.ones(len(joints))
            lower = np.maximum(lower, seed_state - tolerance)
            upper = np.minimum(upper, seed_state + tolerance)
        solver.set_joint_limits(lower, upper)

        (x,y,z), (rx,ry,rz,rw) = base_from_tip
        conf = solver.get_ik(seed_state, x,y,z,rx,ry,rz,rw)
        solver.set_joint_limits(init_lower, init_upper)
        
        return conf
        

















if __name__ == '__main__':
    direct = p.connect(p.GUI)
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
        pepsi = p.loadSDF('can_pepsi/model.sdf')
        pepsi = pepsi[0]
        pyplan.set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=pyplan.stable_z(pepsi, table1))))

    robot = Buff_digit()
    print(pyplan.get_movable_joints(robot.robot))
    time.sleep(200)
