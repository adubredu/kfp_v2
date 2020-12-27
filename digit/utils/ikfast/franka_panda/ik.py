from ..utils import IKFastInfo
from ..ikfast import *

#FRANKA_URDF = "models/franka_description/robots/panda_arm.urdf"
#FRANKA_URDF = "models/franka_description/robots/hand.urdf"
FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"

PANDA_INFO_LEFT = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='left_panda_link0', ee_link='left_panda_link0', free_joints=['panda_joint7'])

PANDA_INFO_RIGHT = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
                        ee_link='panda_link8', free_joints=['panda_joint7'])