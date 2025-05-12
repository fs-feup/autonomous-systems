import numpy as np

import sys
import os

# Add the parent directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import ekf_process_model as pm
import ekf_observation_model as om

class ExtendedKalmanFilter:
    def __init__(self):
        self.x = np.array([0, 0, 0], dtype=float) # 3x1 state vector
        self.P = np.array([[0.1, 0.0, 0.0],
                   [0.0, 0.1, 0.0],
                   [0.0, 0.0, 0.05]], dtype=float) 

        # Diagonal noise matrices
        self.Q = np.diag([0.1, 0.1, 0.5])  # 3x3 
        self.R = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])  # 6x6

    def predict(self, measurement, dt):
        """

        Args:
            measurement (3x1 vector): [ax, ay, angular_velocity]
        """
        # Predict the next state
        self.x = pm.process_model(self.x, measurement, dt)  # Should return 3x1
        if self.x.shape != (3,):
            raise ValueError("Process model output must be a 3x1 vector.")
        
        # Jacobian of the process model
        F = pm.process_model_jacobian(self.x, measurement, dt)  # Should return 3x3
        if F.shape != (3, 3):
            raise ValueError("Jacobian of process model must be a 3x3 matrix.")
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def correct(self, z):
        """
        Update state using the observation z (6x1 vector) [fl_wss, fr_wss, rl_wss, rr_wss, sas, motor_rpm]
        """
        if len(z) != 6:
            raise ValueError("Observation z must be a 6x1 vector.")
        
        # Expected observation and Jacobian
        z_pred = om.observation_model(self.x)  # 6x1
        if z_pred.shape != (6,):
            raise ValueError("Observation model output must be a 6x1 vector.")
        H = om.observation_model_jacobian(self.x)  # 6x3
        if H.shape != (6, 3):
            raise ValueError("Jacobian of observation model must be a 6x3 matrix.")

        # Innovation / residual
        y = z - z_pred

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)  # 3x6

        # State update
        self.x = self.x + K @ y

        # Covariance update using Joseph form (numerically stable)
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    def get_velocities(self):
        return self.x
