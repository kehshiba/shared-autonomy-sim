"""
Robot Teleoperation Demo for Latency and Shared Autonomy
"""

from sim.world import create_world, create_target
from sim.robot_loader import load_robot
from control.teleop import teleop_robot

if __name__ == "__main__":
    physics_client, plane_id = create_world()
    robot_id = load_robot()
    target_id = create_target()
    teleop_robot(robot_id, target_id, delay=1.0)
