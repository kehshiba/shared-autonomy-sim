import pybullet as p
import time
from kinematics.ik import calculate_ik
from kinematics.fk import get_end_effector_pose
from control.latency import LatencyBuffer
from control.assist import assist_command
from control.intent import IntentEstimator

def teleop_robot(robot_id, ee_link_index=11,step_size=0.02,delay=0.3):
    """
    Control Panda end-effector with keyboard
    """
    pos, ori = get_end_effector_pose(robot_id)
    target_pos = list(pos)  # [x, y, z]

    latency_buf = LatencyBuffer(delay_seconds = delay)
    intent_estimator = IntentEstimator(window=6)

    active_command = None  # Last released delayed command

    print("Use W/S: +Y/-Y, A/D: -X/+X, Q/E: +Z/-Z, ESC to quit")    

    while True:
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        keys = p.getKeyboardEvents()
        moved = False

        if ord('w') in keys:
            target_pos[1] += step_size; moved = True
        if ord('s') in keys:
            target_pos[1] -= step_size; moved = True
        if ord('a') in keys:
            target_pos[0] -= step_size; moved = True
        if ord('d') in keys:
            target_pos[0] += step_size; moved = True
        if ord('q') in keys:
            target_pos[2] += step_size; moved = True
        if ord('e') in keys:
            target_pos[2] -= step_size; moved = True
        if 27 in keys:  # ESC
            print("Exiting teleop")
            break

        # Push ONLY when input changed
        if moved:
            intent_estimator.add(list(target_pos))
            latency_buf.push(list(target_pos))

        delayed_command = latency_buf.pop_ready()

        # Represents that a command arrived from network
        if delayed_command is not None:
            active_command = delayed_command
            print("[Latency] Released new command",time.time())
        # Intent estimation
    
        intent = intent_estimator.estimate()

        # Predicted future position
        predicted_pos = list(active_command) if active_command is not None else list(target_pos)
        if intent is not None and active_command is not None:
            predicted_pos = list(active_command + 0.5 * intent)
            print(f"[Prediction] Predicted future pos: {predicted_pos}")

        # Blend human and predicted
        if active_command is not None:
            assisted_pos = assist_command(
                active_cmd=target_pos,
                delayed_cmd=predicted_pos,
                alpha=0.6
            )
            print(f"[Assist] Target: {target_pos}, Assisted: {assisted_pos}")

            joint_angles = calculate_ik(robot_id, assisted_pos, ee_link_index)
            for i, angle in enumerate(joint_angles):
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle
                )
       
        p.stepSimulation()
        time.sleep(1./240.)
