import numpy as np

def update_pose(pose, v, dt):
    """
    Update the pose of the robot using the motion model.
    
    Args:
        pose (np.array): Current pose of the robot [x, y, theta].
        v (float): velocities [m/s] [vx, vy, omega] in the car's body frame!!!
        dt (float): Time step.
    
    Returns:
        np.array: Updated pose [x, y, theta].
    """
    # TODO: Implement the actual motion model
    x_new = 0
    y_new = 0
    theta_new = 0
    return np.array([x_new, y_new, theta_new], dtype=float)