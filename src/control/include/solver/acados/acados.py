import sys
import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, sqrt, atan, atan2, tan, if_else, fabs
from ament_index_python.packages import get_package_prefix
import yaml

path_size = 31
path_point_size = 4  # x, y, velocity, orientation

alpha_max = 0.15
m = 240
Izz = 153
lr = 0.804  # Distance from the center of mass to the rear axle
lf = 0.726  # Distance from the center of mass to the front axle
L = lr + lf
sf = 1.2  # Track width front
sr = 1.2  # Track width rear
s = sf / 2
tire_linear_coefficient = 21.4
tire_lateral_B = 9.63
tire_lateral_C = 1.39
tire_lateral_D = 1.6
tire_lateral_E = 0.95
tire_B = 12.0  # 9.63
tire_C = 1.625 # 1.39
tire_D = 1.775 # 1.60
tire_E = 0.3   # 0.95
ackermann_deviation = 0.0
wheel_radius = 0.203
max_motor_torque = 200  # in Nm
wheel_rotational_inertia = 0.2  # kg*m^2


gravity_acceleration = 9.81

steering_motor_tau = 0.1

def get_config_yaml_path(package_name: str, dir: str, filename: str) -> str:
    """
    Constructs the path to a YAML config file within the workspace.
    
    Args:
        package_name: Name of the ROS package
        dir: Subdirectory within the config folder
        filename: Config filename (without .yaml extension)
    
    Returns:
        Full path to the config YAML file
    """
    try:
        package_prefix = get_package_prefix(package_name)
    except ImportError:
        # Fallback if ament_index_python is not available
        package_prefix = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    workspace_path = os.path.join(package_prefix, "../../config", dir, f"{filename}.yaml")
    return workspace_path

def load_mpc_parameters():
    """
    Load MPC horizon time and steps from the global config YAML file.
    """
    global_config_path = get_config_yaml_path("common_lib", "control", "pacsim")
    
    with open(global_config_path, 'r') as f:
        global_config = yaml.safe_load(f)
    
    mpc_horizon_time = global_config["control"]["mpc_prediction_horizon_seconds"]
    mpc_horizon_steps = global_config["control"]["mpc_prediction_horizon_steps"]
    
    return mpc_horizon_time, mpc_horizon_steps

# def export_mpc_model() -> AcadosModel:
#     model = AcadosModel()
#     model.name = "mpc"
#     # stage-wise reference parameter: [ref_x, ref_y, ref_v, ref_theta]
#     model.p = SX.sym("p", 4)
# 
#     x = SX.sym("x", 13)
#     u = SX.sym("u", 3)
# 
#     # Steering angle at wheels (ackermann geometry). The sign of tan(state[8]) naturally adds or subtracts the track width!
#     sa_fl = atan(L * tan(x[8]) / (L - s * (1 + ackermann_deviation) * tan(x[8])))
#     sa_fr = atan(L * tan(x[8]) / (L + s * (1 + ackermann_deviation) * tan(x[8])))
#     sa_rl = 0
#     sa_rr = 0
# 
#     # Normal load on each tire
#     vertical_load_fl = gravity_acceleration * m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fl - longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
#     vertical_load_fr = gravity_acceleration * m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fr - longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2
#     vertical_load_rl = gravity_acceleration * m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rl + longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
#     vertical_load_rr = gravity_acceleration * m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rr + longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2
# 
#     # Longitudinal velocity of each wheel in the car's frame
#     v_fl_x = x[3] - x[5] * sf / 2
#     v_fr_x = x[3] + x[5] * sf / 2
#     v_rl_x = x[3] - x[5] * sr / 2
#     v_rr_x = x[3] + x[5] * sr / 2
# 
#     # Lateral velocity of each wheel in the car's frame
#     v_fl_y = x[4] + x[5] * lf
#     v_fr_y = x[4] + x[5] * lf
#     v_rl_y = x[4] - x[5] * lr
#     v_rr_y = x[4] - x[5] * lr
# 
#     # Slip angles at each wheel
#     eps_vx = 0.1
#     slip_angle_fl = if_else(fabs(v_fl_x) > eps_vx, sa_fl - atan2(v_fl_y, v_fl_x), 0)
#     slip_angle_fr = if_else(fabs(v_fr_x) > eps_vx, sa_fr - atan2(v_fr_y, v_fr_x), 0)
#     slip_angle_rl = if_else(fabs(v_rl_x) > eps_vx, sa_rl - atan2(v_rl_y, v_rl_x), 0)
#     slip_angle_rr = if_else(fabs(v_rr_x) > eps_vx, sa_rr - atan2(v_rr_y, v_rr_x), 0)
# 
#     # Longitudinal velocity at the frame of each wheel
#     vx_fl = cos(sa_fl) * (v_fl_x) + sin(sa_fl) * (v_fl_y)
#     vx_fr = cos(sa_fr) * (v_fr_x) + sin(sa_fr) * (v_fr_y)
#     vx_rl = cos(sa_rl) * (v_rl_x) + sin(sa_rl) * (v_rl_y)
#     vx_rr = cos(sa_rr) * (v_rr_x) + sin(sa_rr) * (v_rr_y)
# 
#     ## Slip ratios
#     slip_ratio_fl = if_else(fabs(vx_fl) > eps_vx, (wheel_radius * x[9]  - vx_fl) / vx_fl, 0.01 * x[9])
#     slip_ratio_fr = if_else(fabs(vx_fr) > eps_vx, (wheel_radius * x[10] - vx_fr) / vx_fr, 0.01 * x[10])
#     slip_ratio_rl = if_else(fabs(vx_rl) > eps_vx, (wheel_radius * x[11] - vx_rl) / vx_rl, 0.01 * x[11])
#     slip_ratio_rr = if_else(fabs(vx_rr) > eps_vx, (wheel_radius * x[12] - vx_rr) / vx_rr, 0.01 * x[12])
# 
#     # Tire forces
#     fx_fl = (tire_D * vertical_load_fl * sin(tire_C * atan(tire_B * slip_ratio_fl - tire_E * (tire_B * slip_ratio_fl - atan(tire_B * slip_ratio_fl))))) #- rolling_resistance_fl
#     fy_fl = (tire_lateral_D * vertical_load_fl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fl - tire_lateral_E * (tire_lateral_B * slip_angle_fl - atan(tire_lateral_B * slip_angle_fl)))))
#     fx_fr = (tire_D * vertical_load_fr * sin(tire_C * atan(tire_B * slip_ratio_fr - tire_E * (tire_B * slip_ratio_fr - atan(tire_B * slip_ratio_fr))))) #- rolling_resistance_fr
#     fy_fr = (tire_lateral_D * vertical_load_fr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fr - tire_lateral_E * (tire_lateral_B * slip_angle_fr - atan(tire_lateral_B * slip_angle_fr)))))
#     fx_rl = (tire_D * vertical_load_rl * sin(tire_C * atan(tire_B * slip_ratio_rl - tire_E * (tire_B * slip_ratio_rl - atan(tire_B * slip_ratio_rl))))) #- rolling_resistance_rl
#     fy_rl = (tire_lateral_D * vertical_load_rl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rl - tire_lateral_E * (tire_lateral_B * slip_angle_rl - atan(tire_lateral_B * slip_angle_rl)))))
#     fx_rr = (tire_D * vertical_load_rr * sin(tire_C * atan(tire_B * slip_ratio_rr - tire_E * (tire_B * slip_ratio_rr - atan(tire_B * slip_ratio_rr))))) #- rolling_resistance_rr
#     fy_rr = (tire_lateral_D * vertical_load_rr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rr - tire_lateral_E * (tire_lateral_B * slip_angle_rr - atan(tire_lateral_B * slip_angle_rr)))))
# 
#     # FX in the car's frame
#     fx_car_fl = fx_fl * cos(sa_fl) - fy_fl * sin(sa_fl)
#     fx_car_fr = fx_fr * cos(sa_fr) - fy_fr * sin(sa_fr)
#     fx_car_rl = fx_rl * cos(sa_rl) - fy_rl * sin(sa_rl)
#     fx_car_rr = fx_rr * cos(sa_rr) - fy_rr * sin(sa_rr)
#     acceleration_x = (fx_car_fl + fx_car_fr + fx_car_rl + fx_car_rr) / m
# 
#     # FY in the car's frame
#     fy_car_fl = fy_fl * cos(sa_fl) + fx_fl * sin(sa_fl)
#     fy_car_fr = fy_fr * cos(sa_fr) + fx_fr * sin(sa_fr)
#     fy_car_rl = fy_rl * cos(sa_rl) + fx_rl * sin(sa_rl)
#     fy_car_rr = fy_rr * cos(sa_rr) + fx_rr * sin(sa_rr)
#     acceleration_y = (fy_car_fl + fy_car_fr + fy_car_rl + fy_car_rr) / m
# 
#     yaw_moment_fl =   lf * (fy_car_fl) - s * (fx_car_fl)
#     yaw_moment_fr =   lf * (fy_car_fr) + s * (fx_car_fr)
#     yaw_moment_rl = - lr * (fy_car_rl) - s * (fx_car_rl)
#     yaw_moment_rr = - lr * (fy_car_rr) + s * (fx_car_rr)
# 
#     yaw_rate_dot = (yaw_moment_fl + yaw_moment_fr + yaw_moment_rl + yaw_moment_rr) / Izz
#     steering_dot = (u[2] - x[8]) / steering_motor_tau
# 
#     # Demanded torque at wheels
#     demanded_torque_fl = 0 # max_torque_fl * 0
#     demanded_torque_fr = 0 # max_torque_fr * 0
#     demanded_torque_rl = max_motor_torque * u[0] # fmin(max_motor_torque * controls[0], max_torque_rl)
#     demanded_torque_rr = max_motor_torque * u[1] # fmin(max_motor_torque * controls[1], max_torque_rr)
# 
#     # Angular acceleration at wheels
#     alpha_fl = (demanded_torque_fl - wheel_radius * fx_fl) / wheel_rotational_inertia
#     alpha_fr = (demanded_torque_fr - wheel_radius * fx_fr) / wheel_rotational_inertia
#     alpha_rl = (demanded_torque_rl - wheel_radius * fx_rl) / wheel_rotational_inertia
#     alpha_rr = (demanded_torque_rr - wheel_radius * fx_rr) / wheel_rotational_inertia
# 
#     # dynamics
#     f_expl = vertcat(
#         x[3] * cos(x[2]) - x[4] * sin(x[2]),
#         x[3] * sin(x[2]) + x[4] * cos(x[2]),
#         x[5],
#         (u[0] + u[1]) * 80,#acceleration_x + x[5] * x[4]
#         acceleration_y - x[5] * x[3],
#         yaw_rate_dot,
#         SX.zeros(2, 1),
#         steering_dot,
#         alpha_fl,
#         alpha_fr,
#         alpha_rl,
#         alpha_rr,
#     )
# 
#     model.f_expl_expr = f_expl
#     model.x = x
#     model.u = u
#     model.con_h_expr = u[0] - u[1]
# 
#     return model

def export_mpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "mpc"
    # stage-wise reference parameter: [ref_x, ref_y, ref_v, ref_theta]
    model.p = SX.sym("p", 4)

    x = SX.sym("x", 13)
    u = SX.sym("u", 3)

    # dynamics same as before but using MX
    f_expl = vertcat(
        x[3] * cos(x[2]) - x[4] * sin(x[2]),
        x[3] * sin(x[2]) + x[4] * cos(x[2]),
        x[3] * tan(u[2]) / L,
        (u[0] + u[1]) * 80,
        SX.zeros(9, 1),
    )

    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.con_h_expr = u[0] - u[1]

    return model


#def export_mpc_model() -> AcadosModel:
#    model = AcadosModel()
#    model.name = "mpc"
#    # stage-wise reference parameter: [ref_x, ref_y, ref_v, ref_theta]
#    model.p = SX.sym("p", 4)
#
#    x = SX.sym("x", 13)
#    u = SX.sym("u", 3)
#
#    # Steering angle at wheels (ackermann geometry). The sign of tan(state[8]) naturally adds or subtracts the track width!
#    sa_fl = atan(L * tan(x[8]) / (L - s * (1 + ackermann_deviation) * tan(x[8])))
#    sa_fr = atan(L * tan(x[8]) / (L + s * (1 + ackermann_deviation) * tan(x[8])))
#    sa_rl = 0
#    sa_rr = 0
#
#    # Longitudinal velocity of each wheel in the car's frame
#    v_fl_x = x[3] - x[5] * sf / 2
#    v_fr_x = x[3] + x[5] * sf / 2
#    v_rl_x = x[3] - x[5] * sr / 2
#    v_rr_x = x[3] + x[5] * sr / 2
#
#    # Lateral velocity of each wheel in the car's frame
#    v_fl_y = x[4] + x[5] * lf
#    v_fr_y = x[4] + x[5] * lf
#    v_rl_y = x[4] - x[5] * lr
#    v_rr_y = x[4] - x[5] * lr
#
#    # Regularized slip angle calculation to avoid singularities at low speeds
#    eps_vx = 0.1
#    vx_reg_fl = sqrt(v_fl_x**2 + eps_vx**2)
#    vx_reg_fr = sqrt(v_fr_x**2 + eps_vx**2)
#    vx_reg_rl = sqrt(v_rl_x**2 + eps_vx**2)
#    vx_reg_rr = sqrt(v_rr_x**2 + eps_vx**2)
#
#    # Normal load on each tire
#    vertical_load_fl = gravity_acceleration * m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fl - longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
#    vertical_load_fr = gravity_acceleration * m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fr - longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2
#    vertical_load_rl = gravity_acceleration * m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rl + longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
#    vertical_load_rr = gravity_acceleration * m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rr + longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2
#
#    # Slip angles at each wheel
#    slip_angle_fl = sa_fl - atan2(v_fl_y, vx_reg_fl)
#    slip_angle_fr = sa_fr - atan2(v_fr_y, vx_reg_fr)
#    slip_angle_rl = sa_rl - atan2(v_rl_y, vx_reg_rl)
#    slip_angle_rr = sa_rr - atan2(v_rr_y, vx_reg_rr)
#
#    # Longitudinal velocity at the frame of each wheel
#    vx_fl = cos(sa_fl) * (v_fl_x) + sin(sa_fl) * (v_fl_y)
#    vx_fr = cos(sa_fr) * (v_fr_x) + sin(sa_fr) * (v_fr_y)
#    vx_rl = cos(sa_rl) * (v_rl_x) + sin(sa_rl) * (v_rl_y)
#    vx_rr = cos(sa_rr) * (v_rr_x) + sin(sa_rr) * (v_rr_y)
#
#    # Regularized longitudinal velocity for slip ratio calculation
#    vx_fl_for_slip = sqrt(vx_fl**2 + eps_vx**2)
#    vx_fr_for_slip = sqrt(vx_fr**2 + eps_vx**2)
#    vx_rl_for_slip = sqrt(vx_rl**2 + eps_vx**2)
#    vx_rr_for_slip = sqrt(vx_rr**2 + eps_vx**2)
#
#    ## Slip ratios
#    slip_ratio_fl = 0.05 * x[9] #(wheel_radius * x[9] - vx_fl) / vx_fl_for_slip
#    slip_ratio_fr = 0.05 * x[10] #(wheel_radius * x[10] - vx_fr) / vx_fr_for_slip
#    slip_ratio_rl = 0.05 * x[11] #(wheel_radius * x[11] - vx_rl) / vx_rl_for_slip
#    slip_ratio_rr = 0.05 * x[12] #(wheel_radius * x[12] - vx_rr) / vx_rr_for_slip
#
#    # Tire forces
#    fx_fl = (tire_D * vertical_load_fl * sin(tire_C * atan(tire_B * slip_ratio_fl - tire_E * (tire_B * slip_ratio_fl - atan(tire_B * slip_ratio_fl))))) #- rolling_resistance_fl
#    fy_fl = (tire_lateral_D * vertical_load_fl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fl - tire_lateral_E * (tire_lateral_B * slip_angle_fl - atan(tire_lateral_B * slip_angle_fl)))))
#    fx_fr = (tire_D * vertical_load_fr * sin(tire_C * atan(tire_B * slip_ratio_fr - tire_E * (tire_B * slip_ratio_fr - atan(tire_B * slip_ratio_fr))))) #- rolling_resistance_fr
#    fy_fr = (tire_lateral_D * vertical_load_fr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fr - tire_lateral_E * (tire_lateral_B * slip_angle_fr - atan(tire_lateral_B * slip_angle_fr)))))
#    fx_rl = (tire_D * vertical_load_rl * sin(tire_C * atan(tire_B * slip_ratio_rl - tire_E * (tire_B * slip_ratio_rl - atan(tire_B * slip_ratio_rl))))) #- rolling_resistance_rl
#    fy_rl = (tire_lateral_D * vertical_load_rl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rl - tire_lateral_E * (tire_lateral_B * slip_angle_rl - atan(tire_lateral_B * slip_angle_rl)))))
#    fx_rr = (tire_D * vertical_load_rr * sin(tire_C * atan(tire_B * slip_ratio_rr - tire_E * (tire_B * slip_ratio_rr - atan(tire_B * slip_ratio_rr))))) #- rolling_resistance_rr
#    fy_rr = (tire_lateral_D * vertical_load_rr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rr - tire_lateral_E * (tire_lateral_B * slip_angle_rr - atan(tire_lateral_B * slip_angle_rr)))))
#
#    # FX in the car's frame
#    fx_car_fl = fx_fl * cos(sa_fl) - fy_fl * sin(sa_fl)
#    fx_car_fr = fx_fr * cos(sa_fr) - fy_fr * sin(sa_fr)
#    fx_car_rl = fx_rl * cos(sa_rl) - fy_rl * sin(sa_rl)
#    fx_car_rr = fx_rr * cos(sa_rr) - fy_rr * sin(sa_rr)
#    acceleration_x = 0 #(fx_car_fl + fx_car_fr + fx_car_rl + fx_car_rr) / m
#
#    # FY in the car's frame
#    fy_car_fl = fx_fl * sin(sa_fl) + fy_fl * cos(sa_fl)
#    fy_car_fr = fx_fr * sin(sa_fr) + fy_fr * cos(sa_fr)
#    fy_car_rl = fx_rl * sin(sa_rl) + fy_rl * cos(sa_rl)
#    fy_car_rr = fx_rr * sin(sa_rr) + fy_rr * cos(sa_rr)
#    acceleration_y = (fy_car_fl + fy_car_fr + fy_car_rl + fy_car_rr) / m
#
#    yaw_moment_fl =   lf * (fy_car_fl) - s * (fx_car_fl)
#    yaw_moment_fr =   lf * (fy_car_fr) + s * (fx_car_fr)
#    yaw_moment_rl = - lr * (fy_car_rl) - s * (fx_car_rl)
#    yaw_moment_rr = - lr * (fy_car_rr) + s * (fx_car_rr)
#    yaw_rate_dot = (yaw_moment_fl + yaw_moment_fr + yaw_moment_rl + yaw_moment_rr) / Izz
#
#    steering_dot = (u[2] - x[8]) / steering_motor_tau
#
#    # Demanded torque at wheels
#    demanded_torque_fl = 0 # max_torque_fl * 0
#    demanded_torque_fr = 0 # max_torque_fr * 0
#    demanded_torque_rl = max_motor_torque * u[0] # fmin(max_motor_torque * controls[0], max_torque_rl)
#    demanded_torque_rr = max_motor_torque * u[1] # fmin(max_motor_torque * controls[1], max_torque_rr)
#
#    # Angular acceleration at wheels
#    alpha_fl = (demanded_torque_fl - wheel_radius * fx_fl) / wheel_rotational_inertia
#    alpha_fr = (demanded_torque_fr - wheel_radius * fx_fr) / wheel_rotational_inertia
#    alpha_rl = (demanded_torque_rl - wheel_radius * fx_rl) / wheel_rotational_inertia
#    alpha_rr = (demanded_torque_rr - wheel_radius * fx_rr) / wheel_rotational_inertia
#
#    # dynamics
#    f_expl = vertcat(
#        x[3] * cos(x[2]) - x[4] * sin(x[2]),
#        x[3] * sin(x[2]) + x[4] * cos(x[2]),
#        x[5],
#        (u[0] + u[1]) * 80,
#        acceleration_y - x[5] * x[3],
#        yaw_rate_dot,
#        SX.zeros(2, 1),
#        steering_dot,
#        alpha_fl,
#        alpha_fr,
#        alpha_rl,
#        alpha_rr
#    )
#
#    model.f_expl_expr = f_expl
#    model.x = x
#    model.u = u
#    model.con_h_expr = u[0] - u[1]
#
#    return model

def setup_cost_function(ocp: AcadosOcp):
    """
    Sets up the cost function to track a SPECIFIC reference point 'p' per stage.
    """
    # 1. Extract symbolic variables
    x = ocp.model.x
    u = ocp.model.u # <--- WE NEED THIS
    p = ocp.model.p 

    # 2. Configure Cost Type
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # 3. Define the Residuals
    psi = x[2]
    ref_theta = p[3]
    theta_cost_term = 2.0 * (1.0 - cos(psi - ref_theta))

    # --- FIX: Add 'u' to the residuals ---
    # We want u[0] (throttle), u[1] (brake), u[2] (steer) to ideally be 0 (or low)
    # unless necessary to minimize the state error.
    cost_expression = vertcat(
        x[0] - p[0],      # X Position Error
        x[1] - p[1],      # Y Position Error
        x[3] - p[2],      # Velocity Error
        theta_cost_term,  # Orientation Error
        u[0],             # Penalty on Throttle
        u[1],             # Penalty on Brake
        u[2]              # Penalty on Steering
    )

    # Terminal Residual (No controls at the last step)
    cost_expression_e = vertcat(
        x[0] - p[0],
        x[1] - p[1],
        x[3] - p[2],
        theta_cost_term
    )

    ocp.model.cost_y_expr = cost_expression
    ocp.model.cost_y_expr_e = cost_expression_e

    # 4. Define Weight Matrices (W)
    # Weights: [X, Y, V, Theta, Throttle, Brake, Steer]
    # Note: High weight on Steer (e.g. 5.0 or 10.0) prevents shaky steering
    weights = np.array([10.0, 10.0, 1.0, 5.0, 0.1, 0.1, 10.0])
    
    # Terminal weights (Only 4 states)
    weights_e = np.array([10.0, 10.0, 1.0, 5.0])

    ocp.cost.W = np.diag(weights)
    ocp.cost.W_e = np.diag(weights_e)

    # 5. Set Numeric References to Zero
    # ny = 7 (4 states + 3 controls)
    ocp.cost.yref = np.zeros(7)
    ocp.cost.yref_e = np.zeros(4)

    # IMPORTANT: Update Dimensions
    ocp.dims.ny = 7
    ocp.dims.ny_e = 4


def create_ocp_solver(gen_base_dir="./build/acados"):
    c_code_dir = os.path.abspath("./src/control/include/solver/acados/c_generated_code")
    json_path = os.path.abspath(os.path.join(gen_base_dir, "acados_ocp_mpc.json"))

    if os.path.exists(c_code_dir):
        shutil.rmtree(c_code_dir)
    if os.path.exists(json_path):
        os.remove(json_path)

    os.makedirs(os.path.dirname(c_code_dir), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)

    prediction_horizon_seconds, prediction_horizon_steps = load_mpc_parameters()

    ocp = AcadosOcp()
    ocp.model = export_mpc_model()
    ocp.code_export_directory = c_code_dir

    ocp.solver_options.N_horizon = prediction_horizon_steps
    ocp.solver_options.tf = prediction_horizon_seconds
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = 30
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.with_batch_functionality = False
    ocp.solver_options.ext_fun_compile_flags = "-O3 -march=native -ffast-math"

    # Initial state constraint (required for set_state logic)
    ocp.constraints.x0 = np.zeros(13)
    ocp.parameter_values = np.zeros(4)

    setup_cost_function(ocp)

    # Lower bounds for controls
    u_min = np.array([-1.0, -1.0, -0.335])  # adjust these values as needed

    # Upper bounds for controls
    u_max = np.array([1.0, 1.0, 0.335])  # adjust these values as needed

    ocp.constraints.lbu = u_min  # lower bound on u
    ocp.constraints.ubu = u_max  # upper bound on u
    ocp.constraints.idxbu = np.array([0, 1, 2])  # which control inputs have bounds

    max_diff = 0.1  # |u[0] - u[1]| <= max_diff
    ocp.constraints.lh = np.array([-max_diff])  # lower bound
    ocp.constraints.uh = np.array([max_diff])  # upper bound

    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        return solver
    except Exception as e:
        print(f"FAIL: {e}")
        return None


if __name__ == "__main__":
    create_ocp_solver()