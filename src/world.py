import os, sys 
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects, graspa_layouts
import pybullet_planning as pyplan
import numpy as np



def test_elenas_code():
    # Open GUI and set pybullet_data in the path
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setTimeStep(1 / 240.)

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    flags = p.URDF_USE_INERTIA_FROM_FILE
    # obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(), 'YcbBanana', "model.urdf"), [1., 0.0, 0.8], flags=flags)


    ids1 = p.loadSDF(os.path.join(graspa_layouts.getDataPath(), 'layout_0.sdf'))
    ids2 = p.loadSDF(os.path.join(graspa_layouts.getDataPath(), 'layout_1.sdf'))
    ids3 = p.loadSDF(os.path.join(graspa_layouts.getDataPath(), 'layout_2.sdf'))

    p.setGravity(0, 0, -9.8)

    while 1:
        p.stepSimulation()
        time.sleep(1./240)


def vhacd():
    p.connect(p.DIRECT)
    name_in = '/home/bill/garage/kfp_v2/digit/models/bag/bag2.obj'
    name_out = "/home/bill/garage/kfp_v2/digit/models/bag/bag2_vhacd.obj"
    name_log = "log.txt"
    p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=50000 )

class Grocery_World:
    def __init__(self):
        p.setAdditionalSearchPath('../digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbTomatoSoupCan']#, 'YcbMustardBottle', 'YcbPear','YcbPottedMeatCan', 'YcbPowerDrill', 'YcbScissors','YcbStrawberry', 'YcbTennisBall',  'YcbTomatoSoupCan']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        self.xrange = (0.6, 0.9)
        self.yrange = (-0.45, -0.2)
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
            self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
            self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.dining = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
            self.bag = p.loadURDF('bag/bag.urdf',[0.8,0.25,0.9], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.load_objects()
            
        
            
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,0.9], flags=flags)
        
        print(self.ob_idx)


if __name__ == '__main__':
    # vhacd()
    # '''
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setRealTimeSimulation(1)
    # p.setTimeStep(1 / 240.)

    script_path = os.path.dirname(os.path.realpath(__file__))
    os.sys.path.append(os.path.realpath(script_path + '/../'))
    from digit.src.buff_digit import Buff_digit 
    
    

    
    with pyplan.HideOutput(enable=True):
        rid = p.loadURDF('/home/bill/garage/kfp_v2/digit/models/buff_digit/prost_digit_freight.urdf',[0.4,0,0],useFixedBase=True)
        robot = Buff_digit(client, rid) 
        time.sleep(5)
        gw = Grocery_World()
        p.setGravity(0, 0, -9.81)
    # time.sleep(2)
    '''
    for item in gw.ob_idx:
        time.sleep(2)
        banana = gw.ob_idx[item]
        grasp_position, grasp_orientation = robot.get_top_grasp(banana)
        robot.plan_and_execute_arm_motion(grasp_position, grasp_orientation,'left_arm')
        time.sleep(7)
        robot.hold(banana, 'left_arm')
        time.sleep(3)
        bag_position, bag_orientation = [grasp_position[0],grasp_position[1],1.1],p.getQuaternionFromEuler((1.57,1.57,0))
        robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
        time.sleep(5)
        bag_position, bag_orientation = [0.8,0.35,1.2],p.getQuaternionFromEuler((1.57,1.57,0))
        robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
        time.sleep(5)
        # bag_position2, bag_orientation2 = [0.7,0.45,0.8],p.getQuaternionFromEuler((1.57,1.57,0))
        # robot.plan_and_execute_arm_motion(bag_position2, bag_orientation2,'right_arm')
        # time.sleep(5)
        robot.release_hold('left_arm')
        time.sleep(2)
        bag_position, bag_orientation = [0.8,0.35,1.2],p.getQuaternionFromEuler((1.57,1.57,0))
        robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
    '''
    pick_pose = [0.4,-0.25,0]
    place_pose = [0.4,0.25,0]
    dest_poses = [[0.8,0.22,1.1], [0.8,0.24,1.1], [0.8,0.26,1.1],[0.85,0.22,1.1], [0.85,0.24,1.1], [0.85,0.26,1.1]]
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    ind = 0
    for item in gw.ob_idx:
        obj = gw.ob_idx[item]  
        robot.plan_and_drive_to_pose(pick_pose, base_limits,obstacles=[gw.table,gw.bag])
        time.sleep(2)
        try:
            robot.pick_up(obj,'right_arm')
        except:
            continue
        time.sleep(5)

        robot.plan_and_drive_to_pose(place_pose, base_limits,obstacles=[gw.table,gw.bag])
        time.sleep(2)
        robot.place_at(dest_poses[ind],'right_arm')
        print('done')
        ind +=1
        time.sleep(2)
    dining_pose= [-1.0,-1.0,-1.57]
    stove_pose = [-4.0,0.5,-3.1415]
    robot.plan_and_drive_to_pose(dining_pose, base_limits,obstacles=[gw.table,gw.dining])
    time.sleep(2)
    robot.tuck_arm('right_arm',side=True)
    robot.plan_and_drive_to_pose(stove_pose, base_limits,obstacles=[gw.table,gw.dining, gw.kitchen])
    time.sleep(2)
    time.sleep(205)
    # while 1:
    #     p.stepSimulation()
    #     time.sleep(1./240)
    # '''