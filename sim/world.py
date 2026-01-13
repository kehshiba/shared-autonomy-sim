import pybullet as p
import pybullet_data

def create_world():
    physicsClient = p.connect(p.GUI)
    p.setGravity(0,0,-9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    return physicsClient, plane_id