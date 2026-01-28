# Latency-Aware Shared Autonomy Robot Simulation

## Overview
This project simulates a Panda robotic arm in PyBullet with **latency-aware shared autonomy**. It allows keyboard teleoperation of the robot while incorporating:

- **Command latency simulation**: Human input commands are delayed to mimic network or system delays.  
- **Intent estimation**: Predicts the likely next movement of the human operator based on recent inputs.  
- **World modeling**: Uses a Kalman filter to predict the future position of a moving target.  
- **Assisted control**: Blends human input, predicted intent, and target prediction to generate smooth, safe robot motion.  
- **Visualization**: Debug lines show human input → assisted position → predicted target.

## Workflow
- Teleoperation via keyboard (W/A/S/D/Q/E).  
- Predictive assistance that anticipates human motion and moving targets.  
- Latency simulation for delayed command handling.  
- Interactive visualization of robot behavior and predicted positions.

## How to Run
1. Install dependencies (`pybullet`, `numpy`, etc.).  
2. `python -m demo.main`
---

