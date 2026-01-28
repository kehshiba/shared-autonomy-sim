import time
import pybullet as p
import pybullet_data

from sim.world import create_world , create_target
from sim.robot_loader import load_robot 
from kinematics.fk import get_end_effector_pose
from kinematics.ik import calculate_ik
from control.teleop import teleop_robot

if __name__ == "__main__":
    physicsClient , plane_id = create_world()
    robot_id = load_robot()
    target_id = create_target()
    teleop_robot(robot_id,target_id,delay=1.0)
