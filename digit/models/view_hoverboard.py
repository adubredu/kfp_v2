import pybullet as p 
import pybullet_data
from pybullet_planning import get_joint_limits


import time

direct = p.connect(p.GUI)  
delta = 0.01
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath('.')
p.loadURDF('floor/floor.urdf')

# yumi = p.loadURDF('yumi_description/yumi_grippers.urdf', [1,1,0])
base = p.loadURDF('hoverboard/hoverboard.urdf',[1,1,.2])#, useFixedBase=True)
for i in range(p.getNumJoints(base)):
    print('')
    print(p.getJointInfo(base, i))
# time.sleep(1000)

def drive_base(linear_speed, angular_speed):
    global base
    left_wheel_id = 0
    right_wheel_id = 1
    wheel_width = 1.0
    left_vel = linear_speed - angular_speed*(wheel_width*0.5)
    right_vel = linear_speed + angular_speed*(wheel_width*0.5)
    p.setJointMotorControl2(base, left_wheel_id, p.VELOCITY_CONTROL, targetVelocity=-left_vel, force=1000)
    p.setJointMotorControl2(base, right_wheel_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)
    print(-left_vel, right_vel)

while True:
    keys = p.getKeyboardEvents()
    if ord('i') in keys:
        linear_speed = 2.0
        angular_speed = 0
        drive_base(linear_speed, angular_speed)


    elif ord('j') in keys:
        linear_speed = 0.0
        angular_speed = -2.0
        drive_base(linear_speed, angular_speed)


    elif ord('l') in keys:
        linear_speed = 0.0
        angular_speed = 2.0
        drive_base(linear_speed, angular_speed)

    else:
        linear_speed = 0.0
        angular_speed = 0.0
        drive_base(linear_speed, angular_speed)