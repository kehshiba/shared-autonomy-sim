import pybullet as p
import time
from kinematics.ik import calculate_ik
from kinematics.fk import get_end_effector_pose

def teleop_robot(robot_id, ee_link_index=11,step_size=0.02):
    """
    Control Panda end-effector with keyboard
    """
    pos, ori = get_end_effector_pose(robot_id)
    target_pos = list(pos)  # [x, y, z]

    print("Use W/S: +Y/-Y, A/D: -X/+X, Q/E: +Z/-Z, ESC to quit")    

    while True:
        keys = p.getKeyboardEvents()
        
        if ord('w') in keys:
            target_pos[1] += step_size
        if ord('s') in keys:
            target_pos[1] -= step_size
        if ord('a') in keys:
            target_pos[0] -= step_size
        if ord('d') in keys:
            target_pos[0] += step_size
        if ord('q') in keys:
            target_pos[2] += step_size
        if ord('e') in keys:
            target_pos[2] -= step_size
        if 27 in keys:  # ESC key
            print("Exiting teleop")
            break

        # Calculate IK for new target
        joint_angles = calculate_ik(robot_id, target_pos, ee_link_index)

        # Move robot
        for i, angle in enumerate(joint_angles):
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )

        p.stepSimulation()
        time.sleep(1./240.)
