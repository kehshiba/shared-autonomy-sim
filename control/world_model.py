"""World model module for Kalman filtering target tracking."""

import numpy as np

class KalmanTargetTracker:
    """Kalman filter for tracking target position and velocity."""

    def __init__(self, dt=1./240.):
        """Initialize Kalman tracker with state and covariance matrices.

        Args:
            dt: Time step for simulation (default 1/240 seconds).
        """
        # State vector: [x, y, z, vx, vy, vz]
        self.dt = dt
        self.x = np.zeros((6, 1))  # initial state
        self.p_matrix = np.eye(6) * 1.0   # initial covariance  # pylint: disable=invalid-name

        # Motion model: constant velocity
        self.f_matrix = np.array([  # pylint: disable=invalid-name
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # Measurement model: we only measure position
        self.h_matrix = np.array([  # pylint: disable=invalid-name
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        # Covariances
        self.q_matrix = np.eye(6) * 0.001  # process noise  # pylint: disable=invalid-name
        self.r_matrix = np.eye(3) * 0.01   # measurement noise  # pylint: disable=invalid-name

    def update(self, measurement):
        """Update state estimate with new position measurement.

        Args:
            measurement: Position measurement [x, y, z].
        """
        z = np.array(measurement).reshape(3, 1)

        # Prediction step
        self.x = self.f_matrix @ self.x
        self.p_matrix = self.f_matrix @ self.p_matrix @ self.f_matrix.T + self.q_matrix

        # Measurement update
        y = z - self.h_matrix @ self.x
        s_matrix = self.h_matrix @ self.p_matrix @ self.h_matrix.T + self.r_matrix  # pylint: disable=invalid-name
        k_matrix = self.p_matrix @ self.h_matrix.T @ np.linalg.inv(s_matrix)  # pylint: disable=invalid-name

        self.x = self.x + k_matrix @ y
        self.p_matrix = (np.eye(6) - k_matrix @ self.h_matrix) @ self.p_matrix

    def predict(self, steps_ahead=1):
        """Predict target position ahead by steps_ahead time steps.

        Args:
            steps_ahead: Number of time steps to predict ahead (default 1).

        Returns:
            Predicted position as [x, y, z] list.
        """
        f_ahead = np.linalg.matrix_power(self.f_matrix, steps_ahead)  # pylint: disable=invalid-name
        pred = f_ahead @ self.x
        return pred[:3].flatten().tolist()
