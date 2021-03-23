import os, sys 
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects, graspa_layouts
import pybullet_planning as pyplan
import numpy as np
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from digit.src.buff_digit import Buff_digit 
from world import Grocery_World
from PIL import Image 


if __name__ == '__main__':
    # vhacd()
    # '''
    name = sys.argv[1]
    client = p.connect(p.GUI)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setRealTimeSimulation(1)  
    

    
    with pyplan.HideOutput(enable=True):
        rid = p.loadURDF('/home/bill/garage/kfp_v2/digit/models/buff_digit/prost_digit_hover.urdf',[0,0,0.1])
        robot = Buff_digit(client, rid) 
        robot.tuck_arm('right_arm',side=True)
        time.sleep(5)
        gw = Grocery_World()
        p.setGravity(0, 0, -9.81)

    robot.tuck_arm('right_arm',side=True)
    robot.plan_and_drive_to_pose(gw.pick_base_pose,obstacles=[gw.table,gw.bag])
    time.sleep(2)
    images = robot.get_camera_images()
    time.sleep(3)
    rgb = Image.fromarray(images[2])
    rgb.save('../images/'+str(name)+'.png')
    time.sleep(5)