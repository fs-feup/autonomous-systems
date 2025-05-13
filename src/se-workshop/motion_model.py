import numpy as np

def update_pose(pose, v, dt):
    
    x_new = pose[0] + v[0] * dt * np.cos(pose[2]) - v[1] * dt * np.sin(pose[2])
    y_new = pose[1] + v[0] * dt * np.sin(pose[2]) + v[1] * dt * np.cos(pose[2])
    theta_new = pose[2] + v[2] * dt

    return np.array([x_new, y_new, theta_new], dtype=float)