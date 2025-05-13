import numpy as np

# Helper function to calculate wheel speed
def calculate_wheel_speed(velocity_x, angular_velocity, lateral_offset, longitudinal_offset):
    return velocity_x + lateral_offset * angular_velocity + longitudinal_offset * angular_velocity

# Helper function to calculate steering angle
def calculate_steering_angle(velocity_x, angular_velocity, wheelbase):
    if velocity_x == 0:
        return 0.0
    return np.arctan((angular_velocity * wheelbase) / velocity_x)

# Helper function to convert velocity to RPM
def velocity_to_rpm(velocity, wheel_radius):
    if wheel_radius == 0:
        raise ValueError("Wheel radius cannot be zero.")
    angular_velocity = velocity / wheel_radius
    rpm = angular_velocity * (60 / (2 * np.pi))
    return rpm

def observation_model(state):
    velocity_x = state[0]
    velocity_y = state[1]
    angular_velocity = state[2]
    
    # values retrieved from pacsim config file
    fa_cm_distance = 0.804
    ra_cm_distance = 0.726
    axel_width = 1.2
    wheel_radius = 0.203
    wheelbase = fa_cm_distance + ra_cm_distance
    gear_ratio = 4.0
    
    fl_wss = calculate_wheel_speed(velocity_x, angular_velocity, -axel_width / 2, fa_cm_distance)
    fr_wss = calculate_wheel_speed(velocity_x, angular_velocity, axel_width / 2, -fa_cm_distance)
    rl_wss = calculate_wheel_speed(velocity_x, angular_velocity, -axel_width / 2, -ra_cm_distance)
    rr_wss = calculate_wheel_speed(velocity_x, angular_velocity, axel_width / 2, ra_cm_distance)
    
    fl_wss = velocity_to_rpm(fl_wss, wheel_radius)
    fr_wss = velocity_to_rpm(fr_wss, wheel_radius)
    rl_wss = velocity_to_rpm(rl_wss, wheel_radius)
    rr_wss = velocity_to_rpm(rr_wss, wheel_radius)
    
    sas = calculate_steering_angle(velocity_x, angular_velocity, wheelbase)
    
    motor_rpm = velocity_to_rpm(velocity_x, wheel_radius) * gear_ratio
    
    return np.array([fl_wss, fr_wss, rl_wss, rr_wss, sas, motor_rpm], dtype=float) 

def observation_model_jacobian(state):
    velocity_x = state[0]
    velocity_y = state[1]
    angular_velocity = state[2]

    # Values retrieved from pacsim config file
    fa_cm_distance = 0.804
    ra_cm_distance = 0.726
    axel_width = 1.2
    wheel_radius = 0.203
    wheelbase = fa_cm_distance + ra_cm_distance
    gear_ratio = 4.0

    rpm_const = 60 / (2 * np.pi * wheel_radius)

    jac = np.zeros((6, 3))


    # Partial derivatives
    jac[0, 0] = rpm_const  # d(fl_wss)/d(velocity_x)
    jac[0, 2] = rpm_const * (-axel_width / 2 + fa_cm_distance)  # d(fl_wss)/d(angular_velocity)

    jac[1, 0] = rpm_const
    jac[1, 2] = rpm_const * (axel_width / 2 - fa_cm_distance)

    jac[2, 0] = rpm_const
    jac[2, 2] = rpm_const * (-axel_width / 2 - ra_cm_distance)

    jac[3, 0] = rpm_const
    jac[3, 2] = rpm_const * (axel_width / 2 + ra_cm_distance)

    if velocity_x != 0:
        denom = velocity_x**2 + (angular_velocity * wheelbase)**2
        jac[4, 0] = - (angular_velocity * wheelbase) / denom  # d(sas)/d(velocity_x)
        jac[4, 2] = (wheelbase * velocity_x) / denom          # d(sas)/d(angular_velocity)
    else:
        jac[4, 0] = 0.0
        jac[4, 2] = 0.0

    jac[5, 0] = rpm_const * gear_ratio  # d(motor_rpm)/d(velocity_x)
    return jac