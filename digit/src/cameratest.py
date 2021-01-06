import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -10)
p.setTimeStep(0.01)

# Add plane
plane_id = p.loadURDF("plane.urdf")

# Add kuka bot
start_pos = [0, 0, 0.001]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
kuka_id = p.loadURDF("kuka_iiwa/model.urdf", start_pos, start_orientation)

fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
def kuka_camera():
    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(kuka_id, 6)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1) # z-axis
    init_up_vector = (0, 1, 0) # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    img = p.getCameraImage(1000, 1000, view_matrix, projection_matrix)
    return img

# Main loop
while True:
    p.stepSimulation()
    kuka_camera()