import numpy as np

def process_model(state, imu_measurements, dt):
    velocity_x = state[0]
    velocity_y = state[1]
    angular_velocity = state[2] 

    acceleration_x = imu_measurements[0]
    acceleration_y = imu_measurements[1]
    angular_acceleration = imu_measurements[2] 
    
    # Coriolis correction
    coriolis_x = 2 * angular_velocity * velocity_y
    coriolis_y = -2 * angular_velocity * velocity_x

    new_velocity_x = velocity_x + (acceleration_x + coriolis_x) * dt
    new_velocity_y = velocity_y + (acceleration_y + coriolis_y) * dt
    new_angular_velocity = angular_velocity + angular_acceleration * dt

    return np.array([new_velocity_x, new_velocity_y, new_angular_velocity], dtype=float)

def process_model_jacobian(state, imu_measurement, dt):
    
    velocity_x = state[0]
    velocity_y = state[1]
    angular_velocity = state[2]

    jacobian = np.eye(3, dtype=float)

    # Partial derivatives
    jacobian[0, 1] = 2 * angular_velocity * dt  # ∂vx/∂vy
    jacobian[0, 2] = 2 * velocity_y * dt        # ∂vx/∂ω
    jacobian[1, 0] = -2 * angular_velocity * dt # ∂vy/∂vx
    jacobian[1, 2] = -2 * velocity_x * dt       # ∂vy/∂ω
    jacobian[2, 0] = 0                         # ∂ω/∂vx
    jacobian[2, 1] = 0                         # ∂ω/∂vy
    jacobian[2, 2] = 1                         # ∂ω/∂ω
    return jacobian 