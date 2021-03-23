#!/usr/bin/env python3
import math,sys
import pybullet as p
import pybullet_data
from pybullet_rendering import RenderingPlugin
from pybullet_rendering.render.pyrender import PyrRenderer as Renderer
from pybullet_rendering.render.panda3d import P3dRenderer

import numpy as np
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

class Simulation:
    def __init__(self, gui):
        self.gui = gui
        self.paused = False
        self.gravity = True

        self.client_id = p.connect(p.DIRECT)
        renderer = P3dRenderer(multisamples=4)
        plugin = RenderingPlugin(self.client_id , renderer)
        print('loaded')

        p.setGravity(0, 0, -9.81)

        self.time = 0

        # Loading floor
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setAdditionalSearchPath('../models')
        self.plane_index = p.loadURDF('floor/floor.urdf')
        p.changeDynamics(self.plane_index, -1, lateralFriction=1, spinningFriction=-1,
                         rollingFriction=-1, restitution=0.9)

        # Loading robot
        flags = p.URDF_USE_INERTIA_FROM_FILE + p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        

        self.robot_index = p.loadURDF('buff_digit/prost_digit_hover.urdf', [0,0,0], useFixedBase=True)

        self.timestep = 1 / 240

        p.setRealTimeSimulation(0)
        print('before camera')
        self.camera = Camera(self.robot_index, self.client_id)

    def step(self):
        self.time += self.timestep
        p.stepSimulation()
        return self.camera.getImage()


class Camera:
    def __init__(self, robot_index, client_id):
        self.robot_index = robot_index
        self.client_id = client_id
        self.counter = 0

        self.frame_rate = 1

        

    def getImage(self):
        com_p, com_o, _, _, _, _ = p.getLinkState(1, 12)
        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (0, 0, 1) # z-axis
        init_up_vector = (0, -1, 0) # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        # Compute view matrix
        view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)

        # Convert 3x4 projection matrix to 4x4 projection matrix
        fov, aspect, nearplane, farplane = 80, 1.0, 0.1, 100 # TODO realictic vals
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        print('cant get image')
        width, height, colorImage, depthImg, segImg = p.getCameraImage(
            500,
            500,
            view_matrix,
            projection_matrix)
        return colorImage
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break



if __name__ == "__main__":
    s = Simulation(True)

    while True:
        image = s.step()
        cv2.imshow('Grocery Item detection', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break