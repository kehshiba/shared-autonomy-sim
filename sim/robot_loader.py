"""Robot Loader Module """

import pybullet as p
import pybullet_data

def load_robot():
    """ Loads the Franka Panda robot into the simulation """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF(
         "franka_panda/panda.urdf",
         basePosition=[0,0,0],
         useFixedBase=True
    )

    return robot_id
