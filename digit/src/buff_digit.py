import numpy as np 
import time
import pybullet_data
import pybullet as p
import pybullet_planning as pyplan
from pybullet_planning import Pose, Point
# from trac_ik_python.trac_ik import IK
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
from utils.ikfast.digit_ik import solve_ik


class Buff_digit:
    def __init__(self, client, robot_id):
        self.client = client 
        self.id = robot_id
        self.state = "turn"
        self.max_angular_speed = 1.0
        self.max_linear_speed = 2.0
        self.turn_tol = 0.05
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.left_wheel_id = 36
        self.right_wheel_id = 37
        self.wheel_width = 0.8
        self.arms_ee = {'left_arm':13, 'right_arm':26}
        self.arms_base = {'left_arm':'left_jlink0', 
                        'right_arm':'right_jlink0'}
        self.arm_joints = {'left_arm':[3,4,8,10,11,12], 
                           'right_arm':[16,17,21,23,24,25]}
        self.elbow_joints={'left_arm':(8,-1.35), 'right_arm':(21,1.35)}
        self.joint_index_ranges = {'left_arm':(0,6), 
                                   'right_arm':(6,12)} 
        self.grasped = {'left_arm':0, 'right_arm':0}
        self.start_up_robot()
        
    def tuck_arm(self, armname, side=True): 
        right_start_conf = [-1.3587102702612153, -0.9894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if side:
            right_start_conf = [0, -1.1894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        left_start_conf = [0, 1.1894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if armname == 'left_arm':
            self.drive_arm_joints(self.arm_joints[armname], left_start_conf)
        else:
            self.drive_arm_joints(self.arm_joints[armname], right_start_conf)



    def start_up_robot(self):
        self.lowerLimits,self.upperLimits,self.jointRanges,self.restPoses = self.get_joint_ranges()
        self.tuck_arm('left_arm')
        self.tuck_arm('right_arm') 
        self.get_camera_images()


    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff < -np.pi: diff += 2.0*np.pi
        while diff > np.pi: diff -= 2.0*np.pi
        return diff

    def tf_arm_frame(self, pose, armname): 
        basename = self.arms_base[armname]
        baseid = pyplan.link_from_name(self.id, basename)
        base_to_world = pyplan.invert(pyplan.get_link_pose(self.id, baseid))
        base_to_pose = pyplan.multiply(base_to_world, pose)
        return base_to_pose 


    def follow_path(self, path):
        length = len(path)
        for i,pose in enumerate(path):
            self.move_to_pose(pose, last=(i==length-1))
        print('at goal pose')


    def plan_and_drive_to_pose(self, goal_pose, limits,obstacles=[]): 
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        # self.follow_path(path)
        self.drive_along_path(path)


    def drive_along_path(self, path):
        for pose in path:
            pyplan.set_base_values(self.id, pose) 
            time.sleep(0.05)

    
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
            # print(error)
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
                # self.tuck_arms()
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
            # print(self.state)
            self.drive_base(linear_speed, angular_speed)
        return True


    def get_joint_ranges(self, includeFixed=False):
        all_joints = pyplan.get_movable_joints(self.id)
        upper_limits = pyplan.get_max_limits(self.id, all_joints)
        lower_limits = pyplan.get_min_limits(self.id, all_joints)

        jointRanges, restPoses = [], []

        numJoints = p.getNumJoints(self.id)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)

            if includeFixed or jointInfo[3] > -1:
                rp = p.getJointState(self.id, i)[0]
                jointRanges.append(2)
                restPoses.append(rp)

        return lower_limits, upper_limits, jointRanges, restPoses


    def drive_arm_joints(self, joints, jointPoses):
        assert(len(joints)==len(jointPoses))

        for i,j in enumerate(joints):
            p.setJointMotorControl2(bodyIndex=self.id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i], maxVelocity=0.5)#, positionGain=0.001)
            p.stepSimulation()


    def plan_and_execute_arm_motion(self, position, orientation, armname): 
        pose = self.tf_arm_frame((position, orientation), armname)
        gen = solve_ik(pose[0], pose[1],armname)
        a = next(gen)
        conf = next(gen)

        if conf is not None:
            joints = self.arm_joints[armname] 
            self.drive_arm_joints(joints, conf)
        else:
            print('No IK solution found')
 


    def get_top_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[2]+=(height/1.5)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        return position, orientation 

    def get_side_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        width = aabb[1][0] - aabb[0][0]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[0] -=(width/2.0)
        orientation = p.getQuaternionFromEuler((1.57,0,0))
        return position, orientation 

    def hold(self, object_id, armname):
        ee_id = self.arms_ee[armname]
        # self.grasped[armname] = p.createConstraint(self.id, ee_id,object_id,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[0,0,0])
        self.grasped[armname]=pyplan.add_fixed_constraint(object_id, self.id, ee_id)

    def release_hold(self, armname):
        p.removeConstraint(self.grasped[armname])


    def pick_up(self, object_id, armname):
        grasp_position, grasp_orientation = self.get_top_grasp(object_id)
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)
        time.sleep(3) 
        self.hold(object_id, armname) 
        grasp_position[2] += 0.2
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)


    def place_at(self, position, armname):
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        intermediate_position = position; intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        intermediate_position[2]-=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        self.release_hold(armname)
        time.sleep(2)
        intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        # self.tuck_arm(armname)

    def get_camera_images(self): 
        eyeid = pyplan.link_from_name(self.id, 'torso_camera_link')
        fov, aspect, nearplane, farplane = 80, 1.0, 0.01, 100
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        com_p, com_o, _, _, _, _ = p.getLinkState(self.id, eyeid) 
        rot_matrix = p.getMatrixFromQuaternion(com_o)#p.getQuaternionFromEuler((1.5707,0.866,0)))
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 1, 0) # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p+ 0.25 * camera_vector, com_p + 100 * camera_vector, up_vector)
        img = p.getCameraImage(640, 640, view_matrix, projection_matrix)
        return img 

        


















if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
    # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME,0)

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models')
    kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
    with pyplan.HideOutput(enable=True):
        floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)

        robot_id = p.loadURDF('buff_digit/prost_digit_hover.urdf',[0,0,0])
        robot = Buff_digit(client, robot_id)
        time.sleep(5)
        kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        table1 = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=False)
        table2 = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
        # pepsi = p.loadSDF('can_pepsi/model.sdf')
        # pepsi = pepsi[0]
        # pyplan.set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=pyplan.stable_z(pepsi, table1))))
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        pepsi = p.loadURDF("cube_small.urdf")
        pyplan.set_pose(pepsi, Pose(Point(x=0.7, y=-0.3, z=pyplan.stable_z(pepsi, table1))))

    # '''    
    pick_pose = [0,0,0]
    place_pose= [-1.0,-1.0,-1.57]
    stove_pose = [-4.0,0.5,-3.1415 ]
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    for i in range(10):
        robot.plan_and_drive_to_pose(place_pose, base_limits,obstacles=[table1,table2])
        robot.get_camera_images()
        time.sleep(2)
        robot.plan_and_drive_to_pose(pick_pose, base_limits,obstacles=[table1,table2])
        robot.get_camera_images()
        time.sleep(2)
        robot.plan_and_drive_to_pose(stove_pose, base_limits,obstacles=[table1,table2, kitchen])
        robot.get_camera_images()
        time.sleep(2) 

    
    # robot.pick_up(pepsi,'right_arm')
    # time.sleep(5)
    # robot.place_at([0.7, 0, 1.1],'right_arm')
    # print('done')
    # time.sleep(200)
    '''
    while True:
        robot.get_camera_images()
        p.stepSimulation()
    time.sleep(200)
    '''
    time.sleep(200)