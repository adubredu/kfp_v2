import os, sys 
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects, graspa_layouts
import pybullet_planning as pyplan
import numpy as np
from world import Grocery_World, Dining_World, Apartment_World, Grocery_Bag

np.random.seed(0)

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




def grocery_test():
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setRealTimeSimulation(1)
    # p.setTimeStep(1 / 240.)

    script_path = os.path.dirname(os.path.realpath(__file__))
    os.sys.path.append(os.path.realpath(script_path + '/../'))
    from digit.src.buff_digit import Buff_digit 
    
    # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "unolog.mp4")


    
    with pyplan.HideOutput(enable=True):
        
        robot = Buff_digit(client) 
        time.sleep(5)
        gw = Grocery_World()
        p.setGravity(0, 0, -9.81)

    # time.sleep(2)
    '''
    # for item in gw.ob_idx:
    #     time.sleep(2)
    #     banana = gw.ob_idx[item]
    #     grasp_position, grasp_orientation = robot.get_top_grasp(banana)
    #     robot.plan_and_execute_arm_motion(grasp_position, grasp_orientation,'left_arm')
    #     time.sleep(7)
    #     robot.hold(banana, 'left_arm')
    #     time.sleep(3)
    #     bag_position, bag_orientation = [grasp_position[0],grasp_position[1],1.1],p.getQuaternionFromEuler((1.57,1.57,0))
    #     robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
    #     time.sleep(5)
    #     bag_position, bag_orientation = [0.8,0.35,1.2],p.getQuaternionFromEuler((1.57,1.57,0))
    #     robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
    #     time.sleep(5) 
    #     robot.release_hold('left_arm')
    #     time.sleep(2)
    #     bag_position, bag_orientation = [0.8,0.35,1.2],p.getQuaternionFromEuler((1.57,1.57,0))
    #     robot.plan_and_execute_arm_motion(bag_position, bag_orientation,'left_arm')
    '''
    logID = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, 'packing.log')
    pick_base_pose = [0.2,-0.25,0]
    place_base_pose = [0.2,0.25,0]
    obj_dest_poses = [[0.8,0.22,1.1], [0.8,0.24,1.1], [0.8,0.26,1.1],[0.85,0.22,1.1], [0.85,0.24,1.1], [0.85,0.26,1.1],[0.8,0.22,1.1], [0.8,0.24,1.1], [0.8,0.26,1.1],[0.85,0.22,1.1]]
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    ind = 0
    for item in gw.ob_idx:
        obj = gw.ob_idx[item]  
        robot.plan_and_drive_to_pose(pick_base_pose, base_limits,obstacles=[gw.table,gw.bag])
        robot.get_camera_images()
        time.sleep(2)
        try:
            robot.pick_up(obj,'right_arm')
        except:
            continue
        time.sleep(5)

        robot.plan_and_drive_to_pose(place_base_pose, base_limits,obstacles=[gw.table,gw.bag])
        robot.get_camera_images()

        time.sleep(2)
        robot.place_at(obj_dest_poses[ind],'right_arm')
        print('done')
        ind +=1
        time.sleep(2)
    dining_pose= [-1.0,-1.0,-1.57]
    stove_pose = [-4.0,0.5,-3.1415]
    robot.plan_and_drive_to_pose(dining_pose, base_limits,obstacles=[gw.table,gw.dining])
    robot.get_camera_images()

    time.sleep(2)
    robot.tuck_arm('right_arm',side=True)
    robot.plan_and_drive_to_pose(stove_pose, base_limits,obstacles=[gw.table,gw.dining, gw.kitchen])
    robot.get_camera_images() 
    p.stopStateLogging(logID)
    time.sleep(205) 


def dining_world_test():
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setRealTimeSimulation(1)
    # p.setTimeStep(1 / 240.)

    script_path = os.path.dirname(os.path.realpath(__file__))
    os.sys.path.append(os.path.realpath(script_path + '/../'))
    from digit.src.buff_digit import Buff_digit  
    
    with pyplan.HideOutput(enable=True):
        
        robot = Buff_digit(client) 
        # robot.tuck_arm('left_arm',left_side=False)
        time.sleep(5)
        dw = Dining_World()
        p.setGravity(0, 0, -9.81)

    obj_dest_poses = [[0.8,0.22,1], [0.8,0.24,1], [0.8,0.26,1],[0.85,0.22,1], [0.85,0.24,1], [0.85,0.26,1],[0.8,0.22,1.1], [0.8,0.24,1], [0.8,0.26,1],[0.85,0.22,1]]

    dining_pose= [-1.0,-1.0,-1.57]; table1_pose = [-3.0,3.0,1.57]
    table2_pose = [-3.0,7.0,1.57]; table3_pose = [0.0,5.0,1.57] 
    def put_on_tray(item, ind): 
        robot.plan_and_drive_to_pose(dw.pick_base_pose, dw.base_limits,obstacles=[dw.main_table])
        time.sleep(2)
        robot.pick_up(dw.ob_idx[item],'right_arm')
        time.sleep(2)
        robot.plan_and_drive_to_pose(dw.place_base_pose, dw.base_limits,obstacles=[dw.main_table])
        time.sleep(2)
        # ind=np.random.randint(10)
        robot.place_at(obj_dest_poses[ind],'right_arm') 

    def send_item_to_table(item, table):
        tables = [table1_pose,table2_pose,table3_pose]
        table_pose = tables[table]
        path = robot.plan_to_pose(table_pose, dw.base_limits,obstacles=[dw.main_table, dw.dtable1,dw.dtable2, dw.dtable3])
        time.sleep(2)
        put_on_tray(item[0],0)
        # pyplan.add_fixed_constraint(dw.tray,dw.ob_idx[item])
        
        put_on_tray(item[1],1) 
        
        time.sleep(2)
        robot.plan_and_execute_arm_motion([0.8,0.14,0.93],p.getQuaternionFromEuler([0,1.57,0]),'right_arm') 
        time.sleep(5) 
        robot.hold(dw.tray, 'right_arm')
        robot.hold(dw.ob_idx[item[0]],'right_arm')
        robot.hold(dw.ob_idx[item[1]], 'right_arm')
        robot.plan_and_execute_arm_motion([0.8,0.14,1.1],p.getQuaternionFromEuler([0,1.57,0]),'right_arm')
        time.sleep(2)
        robot.drive_along_path(path)
        time.sleep(3)
        robot.release_specific_hold(dw.tray, 'right_arm')
        robot.release_specific_hold(dw.ob_idx[item[0]], 'right_arm')
        robot.release_specific_hold(dw.ob_idx[item[1]], 'right_arm')

    def set_table(item):
        ee_pose = pyplan.get_link_pose(robot.id, robot.arms_ee['right_arm'])
        robot.pick_up(dw.ob_idx[item[0]], 'right_arm')
        time.sleep(2)
        place_pose = list(ee_pose[0]) 
        place_pose[0]+=0.1
        place_pose[1]+=0.1
        place_pose[2]-=0.1
        robot.place_at(place_pose , 'right_arm') 
        time.sleep(3)

        robot.pick_up(dw.ob_idx[item[1]], 'right_arm')
        time.sleep(2)
        place_pose2 = list(ee_pose[0]) 
        place_pose2[0]+=0.1
        place_pose2[1]-=0.1
        place_pose2[2]-=0.1
        robot.place_at(place_pose2 , 'right_arm') 

    def go_back_to_counter():
        robot.pick_up_tray(dw.tray,'right_arm')
        time.sleep(2)
        robot.plan_and_drive_to_pose(dw.place_base_pose, dw.base_limits,obstacles=[dw.main_table])
        time.sleep(2)
        robot.place_at([0.8,0.25,0.95],'right_arm')
        time.sleep(2) 

    send_item_to_table(['YcbStrawberry','YcbPottedMeatCan'],1)
    time.sleep(2)
    set_table(['YcbStrawberry','YcbPottedMeatCan'])
    go_back_to_counter()
    send_item_to_table(['YcbGelatinBox','YcbMasterChefCan'],0)
    time.sleep(2)
    set_table(['YcbGelatinBox','YcbMasterChefCan'])
    go_back_to_counter()
    # p.stopStateLogging()
    time.sleep(100)


def test_grocery_bag():
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setRealTimeSimulation(1) 

    script_path = os.path.dirname(os.path.realpath(__file__))
    os.sys.path.append(os.path.realpath(script_path + '/../'))
    from digit.src.buff_digit import Buff_digit  
    
    with pyplan.HideOutput(enable=True):
        
        robot = Buff_digit(client)  
        time.sleep(5)
        gw = Grocery_World()
        gb = Grocery_Bag(gw.bag)
        p.setGravity(0, 0, -9.81)


    items = ['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbPear']

    for item in items:
        item_id = gw.ob_idx[item]
        placement = gb.add_item(item_id)
        if placement is None:
            print('Stopping packing')
            break 
       
        robot.plan_and_drive_to_pose(gw.pick_base_pose, gw.base_limits,obstacles=[gw.table])
        time.sleep(2)
        robot.pick_up(item_id,'right_arm')
        time.sleep(2)
        robot.plan_and_drive_to_pose(gw.place_base_pose, gw.base_limits,obstacles=[gw.table])

        (x,y,z) = placement
        robot.place_at([x,y,z],item_id,'right_arm') 
        time.sleep(2)

    #Remove strawberry
    robot.plan_and_drive_to_pose(gw.place_base_pose, gw.base_limits,obstacles=[gw.table])
    time.sleep(2)
    robot.pick_up(gw.ob_idx['YcbStrawberry'],'right_arm')
    gb.remove_item(gw.ob_idx['YcbStrawberry'])
    time.sleep(2)
    robot.plan_and_drive_to_pose(gw.pick_base_pose, gw.base_limits,obstacles=[gw.table])
    time.sleep(2)
    pl = gw.get_random_table_space()
    robot.place_at(pl,gw.ob_idx['YcbStrawberry'],'right_arm') 
    time.sleep(2)

    #put in mustard
    item_id = gw.ob_idx['YcbPear']
    robot.pick_up(item_id,'right_arm')
    time.sleep(2)
    robot.plan_and_drive_to_pose(gw.place_base_pose, gw.base_limits,obstacles=[gw.table])
    placement = gb.add_item(item_id)
    if placement is not None: 
        (x,y,z) = placement
        robot.place_at([x,y,z],item_id,'right_arm') 
        time.sleep(2)
        time.sleep(100)


def apartment_test():
    client = p.connect(p.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    script_path = os.path.dirname(os.path.realpath(__file__))
    os.sys.path.append(os.path.realpath(script_path + '/../'))
    from digit.src.buff_digit import Buff_digit  
    
    with pyplan.HideOutput(enable=True):
        p.setGravity(0, 0, -9.81)
        aw = Apartment_World()
        robot = Buff_digit(client)
        # pyplan.set_pose(robot.id, pyplan.Pose(pyplan.Point(x=8,y=-2.5,z=0.05)))
        time.sleep(5)  
        print(pyplan.get_pose(robot.id))
        # time.sleep(5) 
    
    p.setRealTimeSimulation(1) 

    pose1 = [8,-2.5,0]
    pose2 = [-6,0,1.57]
    center = [0,0,0] 
    orig = pyplan.get_base_values(robot.id)
    base_limits = ((-60, -60), (60, 60))

    pyplan.set_base_values(robot.id, center)
    path_center_to_go  = robot.plan_to_pose(pose2,base_limits,obstacles=aw.apartment_bodies)

    pyplan.set_base_values(robot.id, pose2)
    path_go_to_center  = robot.plan_to_pose(center,base_limits,obstacles=aw.apartment_bodies)

    pyplan.set_base_values(robot.id, center)
    path_center_to_come = robot.plan_to_pose(pose1,base_limits,obstacles=aw.apartment_bodies)

    pyplan.set_base_values(robot.id, pose1)
    path_come_to_center = robot.plan_to_pose(center,base_limits,obstacles=aw.apartment_bodies)

    pyplan.set_base_values(robot.id, orig)
    for i in range(10): 
        robot.drive_along_path(path_center_to_go)
        time.sleep(1) 
        robot.drive_along_path(path_go_to_center)
        time.sleep(1)
        robot.drive_along_path(path_center_to_come)
        time.sleep(1)
        robot.drive_along_path(path_come_to_center)
        time.sleep(1)
    time.sleep(1000)



def vhacd():
    p.connect(p.DIRECT)
    name_in = '/home/bill/garage/kfp_v2/src/models/aws_robomaker_residential_KitchenCabinet_01/meshes/object.obj'
    name_out = "/home/bill/garage/kfp_v2/src/models/aws_robomaker_residential_KitchenCabinet_01/meshes/object_vhacd.obj"
    name_log = "log.txt"
    p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=50000 )



if __name__ == '__main__': 
    apartment_test() 
    # vhacd()