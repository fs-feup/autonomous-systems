import numpy as np

def observation_model(state):
    """
    Observation model for the state to measurement mapping.
    TODO: Implement the actual observation model.
    
    return: [fl_wss, fr_wss, rl_wss, rr_wss, sas, motor_rpm] predicted from the state.
    """
    
    return np.array([0, 0, 0, 0, 0, 0], dtype=float)  # Replace with actual observation model logic

def observation_model_jacobian(state):
    """
    Jacobian of the observation model.
    TODO: Implement the actual Jacobian of the observation model.
    """
    
    return np.eye(6, 3, dtype=float)  # Replace with actual Jacobian logic
