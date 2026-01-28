import numpy as np

class KalmanTargetTracker:
    def __init__(self, dt=1./240.):
        # State vector: [x, y, z, vx, vy, vz]
        self.dt = dt
        self.x = np.zeros((6, 1))  # initial state
        self.P = np.eye(6) * 1.0   # initial covariance

        # Motion model: constant velocity
        self.F = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])

        # Measurement model: we only measure position
        self.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ])

        # Covariances
        self.Q = np.eye(6) * 0.001  # process noise
        self.R = np.eye(3) * 0.01   # measurement noise

    def update(self, measurement):
        z = np.array(measurement).reshape(3,1)

        # Prediction step
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Measurement update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def predict(self, steps_ahead=1):
        F_ahead = np.linalg.matrix_power(self.F, steps_ahead)
        pred = F_ahead @ self.x
        return pred[:3].flatten().tolist()
