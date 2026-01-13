# kinematics/ik.py
import pybullet as p

def calculate_ik(robot_id, target_pos, ee_link_index=11):
    """
    Returns joint angles to reach target_pos
    """
    joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ee_link_index,
        targetPosition=target_pos
    )
    return joint_angles
