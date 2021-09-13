import os, sys 
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects, graspa_layouts
import pybullet_planning as pyplan
import numpy as np
from pybullet_utils import gazebo_world_parser



class Grocery_World_old:
    def __init__(self,seed=0):
        np.random.seed(seed)
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbPear', 'YcbTomatoSoupCan', 'YcbTennisBall', 'YcbScissors']#,'YcbPowerDrill',  'YcbMustardBottle']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        p.setGravity(0, 0, -9.81)
        self.xrange = (0.71, 0.73)
        self.yrange = (-0.45, -0.15)
        self.base_limits = ((-22.5, -22.5), (22.5, 22.5))
        self.pick_base_pose = [0.2,-0.25,0]
        self.place_base_pose = [0.2,0.25,0]
        self.init_item_properties()
        
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
            # self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
            self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.dining = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
            self.bag = p.loadURDF('bag/bag.urdf',[0.8,0.25,0.9], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.load_objects()
        
            
    def init_item_properties(self):
        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()]) 
            
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,0.92], flags=flags)
        
        print(self.ob_idx)
        self.idx_obs = dict([(value, key) for key, value in self.ob_idx.items()]) 
        print(self.idx_obs)


    def get_random_table_space(self):
        x = np.random.uniform(self.xrange[0], self.xrange[1])
        y = np.random.uniform(self.yrange[0], self.yrange[1])
        z = 1.0
        return [x,y,z]


    def get_item_mass(self, itemname): 
        return self.masses[itemname]


    def get_id(self, itemname):
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden

    def get_name(self, obid):
        return self.real_name[self.idx_obs[obid]]

    def get_names_of_ids(self, ids):
        idx = []
        for idd in ids:
            idx.append(self.real_name[self.idx_obs[idd]])
        return idx


class Grocery_World: #For video
    def __init__(self,seed=0):
        np.random.seed(seed)
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)          
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbPear', 'YcbTomatoSoupCan', 'YcbTennisBall', 'YcbScissors']#,'YcbPowerDrill',  'YcbMustardBottle'] 
        p.setGravity(0, 0, -9.81)
        self.xrange = (0.71, 0.73)
        self.yrange = (-0.45, -0.15)
        self.base_limits = ((-22.5, -22.5), (22.5, 22.5))
        self.pick_base_pose = [0.2,-0.25,0]
        self.place_base_pose = [0.2,0.25,0]
        self.init_item_properties()
        
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/black_floor.urdf',useFixedBase=True) 
            self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True) 
            self.bag = p.loadURDF('bag/bag.urdf',[0.8,0.25,0.88], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.load_objects()

        
            
    def init_item_properties(self):
        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()]) 
            
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,0.92], flags=flags)
        
        print(self.ob_idx)
        self.idx_obs = dict([(value, key) for key, value in self.ob_idx.items()]) 
        print(self.idx_obs)


    def get_random_table_space(self):
        x = np.random.uniform(self.xrange[0], self.xrange[1])
        y = np.random.uniform(self.yrange[0], self.yrange[1])
        z = 1.0
        return [x,y,z]


    def get_item_mass(self, itemname): 
        return self.masses[itemname]


    def get_id(self, itemname): 
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden

    def get_name(self, obid):
        return self.real_name[self.idx_obs[obid]]

    def get_names_of_ids(self, ids):
        idx = []
        for idd in ids:
            idx.append(self.real_name[self.idx_obs[idd]])
        return idx



class Dining_World:
    def __init__(self):
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbMustardBottle', 'YcbPear']#,  'YcbTomatoSoupCan', 'YcbTennisBall', 'YcbPowerDrill', 'YcbScissors']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        self.xrange = (0.6, 0.9)
        self.yrange = (-0.45, -0.2)
        self.base_limits = ((-22.5, -22.5), (22.5, 22.5))
        self.pick_base_pose = [0.2,-0.25,0]
        self.place_base_pose = [0.2,0.25,0]
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
            self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
            self.main_table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.wash_table = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=True)
            self.washbowl= p.loadURDF('table/table.urdf',[-1.0,-2.0,0.9], useFixedBase=False)
            self.dtable1 = p.loadURDF('dinner_table/dinner_table.urdf',[-3.0,4.0,0.4], useFixedBase=True)
            self.dtable2 = p.loadURDF('dinner_table/dinner_table.urdf',[-3.0,8.0,0.4], useFixedBase=True)
            self.dtable3 = p.loadURDF('dinner_table/dinner_table.urdf',[0.0,6.0,0.4], useFixedBase=True)
            self.tray = p.loadURDF('tray/tray.urdf',[0.8,0.25,0.9],p.getQuaternionFromEuler([0,0,-1.57]) )
            self.load_objects()
        self.object_indices = {"pear":self.ob_idx["YcbPear"],
        "tray":self.tray, "wash-station":self.wash_table, "stove-station":self.kitchen, "wash-bowl":self.wash_table, "stove":self.ktichen}
            
             
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,1], flags=flags)
        
        print(self.ob_idx)


    def sample_pose_from_normal(mean_pose, sigma, num_particles):
        xs = np.random.normal(loc=mean_pose[0][0], scale=sigma ,size=num_particles)
        ys = np.random.normal(loc=mean_pose[0][1], scale=sigma ,size=num_particles)
        zs = np.random.normal(loc=mean_pose[0][2], scale=sigma ,size=num_particles)
        positions = [(x,y,z) for x,y,z in zip(xs,ys,zs)]
        quats = [mean_pose[1] for _ in range(num_particles)]
        samples = [(pose, qts) for pose,qts in zip(positions, quats)]
        return samples


    def sample_se2_from_normal(mean, sigma, num_particles):
        xs = np.random.normal(loc=mean[0],scale=sigma, size=num_particles)
        ys = np.random.normal(loc=mean[1],scale=sigma, size=num_particles)
        positions = [(x,y,mean[2]) for x,y in zip(xs,ys)]
        return positions


    def get_prior_belief(self, num_particles, targets,sigma=0.1):
        prior = {}
        for item in targets:
            if item == "pear":
                mean_pose = pyplan.get_pose(self.object_indices['pear'])
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['pear'] = particles 

            elif item == 'wash-station':
                mean_pose = [-1.0,-1.0,-1.57]
                samples = self.sample_se2_from_normal(mean_pose, sigma, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-station'] = particles

            elif item == 'stove-station':
                mean_pose = [-4.0,0.5,-3.1415 ]
                samples = self.sample_se2_from_normal(mean_pose, sigma, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove-station'] = particles

            elif item == 'tray':
                mean_pose = pyplan.get_pose(self.tray)
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['tray'] = particles

            elif item == 'wash-bowl':
                mean_pose = pyplan.get_pose(self.washbowl)
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-bowl'] = particles

            elif item == 'stove':
                mean_pose = [[-4.1,0.5,0.9 ],[0,0,0,1]]
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove'] = particles 
        return prior 



class Apartment_World:
    def __init__(self, seed=0):
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir) 
        p.setAdditionalSearchPath(model_path+'/digit/models')
        floor = p.loadURDF('floor/black_floor.urdf',[0,0,0.05],useFixedBase=True)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        self.apartment_bodies = gazebo_world_parser.parseWorld( p, filepath =  "worlds/small_house.world")
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)







                



class Grocery_Bag:
    def __init__(self, bagid):
        self.id = bagid
        self.full_cpty = 4
        w = 0.25
        b = 0.2
        x, y, _ = pyplan.get_point(bagid)
        self.index = 0
        # denom = 10
        denom = 5
        self.positions = [(x+b/denom, y+w/denom), (x+b/denom, y-w/denom), (x-b/denom, y+w/denom), (x-b/denom,y-w/denom)]
        self.radius = 0.00625
        self.z = 0.97
        self.occupancy = [0,0,0,0]
        self.items_added = {}
        self.bag_scene_graph = {}
        self.num_base_items = 0
        self.num_total_items = 0

    def add_item(self, itemid):
        if self.num_base_items >= self.full_cpty:
            print('Bag full')
            return None
        cx, cy = self.positions[self.index]
        x = np.random.uniform(cx-self.radius, cx+self.radius)
        y = np.random.uniform(cy-self.radius, cy+self.radius)
        self.items_added[itemid] = self.index
        self.bag_scene_graph[itemid] = None
        self.occupancy[self.index] = 1 
        self.num_base_items +=1
        self.num_total_items +=1
        self.reposition_index()
        return (x,y,self.z)

    def add_on_item(self, topid, botid):
        self.items_added[topid] = 99
        self.bag_scene_graph[botid] = topid
        self.num_total_items +=1

    def remove_on_item(self, itemid):
        if itemid in self.items_added:
            self.items_added.pop(itemid)
            self.num_total_items -=1
            botid = self.get_item_under(itemid)
            self.bag_scene_graph[botid] = None

    def remove_item(self, itemid):
        if itemid in self.items_added:
            index = self.items_added[itemid]
            if index == 99:
                return
            self.occupancy[index] = 0
            self.items_added.pop(itemid)
            self.num_base_items -=1
            self.num_total_items -=1
            self.reposition_index()

    def is_box_full(self):
        return self.num_base_items == self.full_cpty
        
    def reposition_index(self):
        free = False
        for i in range(len(self.occupancy)):
            if self.occupancy[i] == 0:
                self.index = i 
                free = True
                break
        if not free:
            print('1 free space left')

    def get_ids_of_items(self):
        names = self.items_added.keys() 
        return names

    def get_itemid_on_top(self, obid):
        if obid in self.bag_scene_graph:
            topid = self.bag_scene_graph[obid]
            return topid
        #     if topid is None:
        #         return None
        #     else:
        #         return self.gw.get_name(topid)
        # else:
        #     return None

    def get_item_under(self, obid):
        for bot in self.bag_scene_graph:
            if self.bag_scene_graph[bot] == obid:
                return bot 
        return None










if __name__ == '__main__':
    # vhacd()
    # '''
    dining_world_test()
    # '''
