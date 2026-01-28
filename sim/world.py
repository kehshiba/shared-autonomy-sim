""" World Creation Module """

import pybullet as p
import pybullet_data

def create_world():
    """ Sets up the simulation world with a plane and gravity """
    physics_client = p.connect(p.GUI)
    p.setGravity(0,0,-9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    return physics_client, plane_id

def create_target():
    """ Creates a target object in the simulation """
    # Create a target sphere
    target_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                        radius=0.05,
                                        rgbaColor=[1,0,0,1])
    target_collision = p.createCollisionShape(shapeType=p.GEOM_SPHERE,
                                              radius=0.05)
    target_pos = [0.5, 0.2, 0.5]  # example target
    target_id = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=target_collision,
                                  baseVisualShapeIndex=target_visual,
                                  basePosition=target_pos)
    
    return target_id
