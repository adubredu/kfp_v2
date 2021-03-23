import pybullet as p
from pybullet_utils import gazebo_world_parser
import time

# p.connect(p.GUI)
p.connect(p.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

models = gazebo_world_parser.parseWorld( p, filepath = "worlds/small_house.world")
print(models)
# p.loadSDF('models/aws_robomaker_residential_ShoeRack_01/model.sdf')
    
shadowIntensityParam = p.addUserDebugParameter("shadowIntensity", 0, 1, 0)

p.configureDebugVisualizer(shadowMapResolution = 8192)
p.configureDebugVisualizer(shadowMapWorldSize = 25)
 
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
dt = 1./240.

while (1):
  shadowMapIntensity = p.readUserDebugParameter(shadowIntensityParam)
  p.configureDebugVisualizer(shadowMapIntensity=shadowMapIntensity)
  p.stepSimulation()
  p.getCameraImage(640,480, renderer=p.ER_BULLET_HARDWARE_OPENGL )
  time.sleep(dt)