import pybullet as p
import pybullet_data

def load_robot():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    robot_id = p.loadURDF(
         "franka_panda/panda.urdf",
         basePosition=[0,0,0],
         useFixedBase=True
    )

    return robot_id
