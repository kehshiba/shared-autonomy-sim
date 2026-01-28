""" Forward Kinematics Module """
import pybullet as p

def get_end_effector_pose(robot_id, ee_link_index=11):
    """
    Returns end-effector position and orientation
    ee_link_index: for Panda hand, 11 is commonly end-effector
    """
    state = p.getLinkState(robot_id, ee_link_index)
    position = state[4]  # world position of link frame
    orientation = state[5]  # quaternion
    return position, orientation
