import time
import pybullet as p
import pybullet_data

from sim.world import create_world
from sim.robot_loader import load_robot
from kinematics.fk import get_end_effector_pose
from kinematics.ik import calculate_ik

if __name__ == "__main__":
    physicsClient , plane_id = create_world()
    robot_id = load_robot()

    print("Robot ID:", robot_id)

    num_joints = p.getNumJoints(robot_id)
    
    print("Number of joints",num_joints)

    pos, ori = get_end_effector_pose(robot_id)
    print("Initial end-effector position:", pos)

    target_pos = [0.4, 0.0, 0.4]  # in world frame
    joint_angles = calculate_ik(robot_id, target_pos)

    for i, angle in enumerate(joint_angles):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle
        )

    # Step simulation to see robot move
    for _ in range(240*5):  # ~3 seconds
        p.stepSimulation()
        time.sleep(1./240.)

    # Check new FK
    pos, ori = get_end_effector_pose(robot_id)
    print("New end-effector position:", pos)