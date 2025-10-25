import numpy as np

def process_model(state, imu_measurements, dt):
    """
    Process model for the velocity state transition.
    TODO: Implement the actual process model (hint: look into Coriolis effect).
    """
    
    return np.array([0,0,0], dtype=float)  # Replace with actual process model logic

def process_model_jacobian(state, imu_measurement, dt):
    """
    Jacobian of the process model.
    TODO: Implement the actual Jacobian of the process model.
    """
    
    return np.eye(3,3, dtype=float)  # Replace with actual Jacobian logic
