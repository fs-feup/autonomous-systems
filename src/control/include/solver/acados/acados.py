import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import (SX, vertcat, sin, cos, horzcat, sqrt, fmin, tan, if_else, exp, sum, atan, fmax, atan2, tanh, log)

# Car parameters
path_size = 50
tire_B = 9.63
tire_C = -1.39
tire_D = 1.6
tire_E = 1.0
tire_lateral_B = 9.63
tire_lateral_C = -1.39
tire_lateral_D = 1.6
tire_lateral_E = 1.0
tire_self_aligning_B = 9.63
tire_self_aligning_C = -1.39
tire_self_aligning_D = 1.6
tire_self_aligning_E = 1.0
lr = 0.804  # Distance from the center of mass to the rear axle
lf = 0.726  # Distance from the center of mass to the front axle
L = lr + lf  # Wheelbase
sf = 1.2  # Track width front
sr = 1.2  # Track width rear
s = sf / 2 # Half track width
ackermann_deviation = 0.0  # Ackermann deviation factor: 0.0 is perfect ackermann, >0 is over ackermann, <0 is under ackermann
h_cg = 0.245  # Height of center of gravity
cla = -0.86
cda = 0.73
aero_area = 0.44
cp_location = 0  # Center of pressure location from the CG (positive is forward)
m = 200.707
Izz = 101.082
wheel_radius = 0.203
gear_ratio = 4.0
inner_steering_ratio = 0.1869
outer_steering_ratio = 0.1582
air_density = 1.225
gravity_acceleration = 9.81
roll_stiffness_front = 15000
roll_stiffness_rear = 15000
max_motor_power = 32500 # in Watts
max_motor_torque = 200  # in Nm
rated_motor_rpms = 5000  # in RPM -> under this speed max torque is available (limited by current), above this speed power is limited by max power
wheel_rotational_inertia = 5.0  # kg*m^2
steering_gain = 35
steering_damping = 10
rolling_resistance_coefficient = 0.015
steering_motor_tau = 0.1  # Time constant for steering motor response
path_point_size = 4  # x, y, velocity, orientation
distance_weight = 1.0
velocity_weight = 1.0
orientation_weight = 1.0
u_rl_weight = 1e-2
u_rr_weight = 1e-2
u_steer_weight = 1e-3



def export_mpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "mpc"
    model.p_global = SX.sym(
        "p_global", path_size * path_point_size
    )  # Path: 50 points x 4 values each

    # State: [x, y, yaw, vx, vy, yaw_rate, ax, ay, steering_angle_at_steering_wheel, fl_angular_speed, fr_angular_speed, rl_angular_speed, rr_angular_speed]
    state = SX.sym("state", 13)
    # Controls: [torque_rear_left, torque_rear_right, steering_angle_command]
    controls = SX.sym("controls", 3)

    # Steering angle at wheels (ackermann geometry). The sign of tan(state[8]) naturally adds or subtracts the track width!
    #sa_fl = state[8] # atan(L * tan(state[8]) / (L - s * (1 + ackermann_deviation) * tan(state[8])))
    #sa_fr = state[8] # atan(L * tan(state[8]) / (L + s * (1 + ackermann_deviation) * tan(state[8])))
    #sa_rl = 0
    #sa_rr = 0

    ## Longitudinal velocity of each wheel in the car's frame
    # v_fl_x = state[3] - state[5] * sf / 2
    # v_fr_x = state[3] + state[5] * sf / 2
    # v_rl_x = state[3] - state[5] * sr / 2
    # v_rr_x = state[3] + state[5] * sr / 2

    ## Lateral velocity of each wheel in the car's frame
    #v_fl_y = state[4] + state[5] * lf
    #v_fr_y = state[4] + state[5] * lf
    #v_rl_y = state[4] - state[5] * lr
    #v_rr_y = state[4] - state[5] * lr

    # Regularized slip angle calculation to avoid singularities at low speeds
    # eps_vx = 0.1
    # vx_reg_fl = sqrt(v_fl_x**2 + eps_vx**2)
    # vx_reg_fr = sqrt(v_fr_x**2 + eps_vx**2)
    # vx_reg_rl = sqrt(v_rl_x**2 + eps_vx**2)
    # vx_reg_rr = sqrt(v_rr_x**2 + eps_vx**2)

    # Slip angles at each wheel
    slip_angle_fl = 0.1 * state[8] # sa_fl - atan2(v_fl_y, vx_reg_fl)
    slip_angle_fr = 0.1 * state[8] # sa_fr - atan2(v_fr_y, vx_reg_fr)
    slip_angle_rl = 0 # sa_rl - atan2(v_rl_y, vx_reg_rl)
    slip_angle_rr = 0 # sa_rr - atan2(v_rr_y, vx_reg_rr)
#
    ## Longitudinal velocity at the frame of each wheel
    # vx_fl = cos(sa_fl) * (v_fl_x) + sin(sa_fl) * (v_fl_y)
    # vx_fr = cos(sa_fr) * (v_fr_x) + sin(sa_fr) * (v_fr_y)
    # vx_rl = cos(sa_rl) * (v_rl_x) + sin(sa_rl) * (v_rl_y)
    # vx_rr = cos(sa_rr) * (v_rr_x) + sin(sa_rr) * (v_rr_y)
#
    ## Regularized longitudinal velocity for slip ratio calculation
    # vx_fl_for_slip = sqrt(vx_fl**2 + eps_vx**2)
    # vx_fr_for_slip = sqrt(vx_fr**2 + eps_vx**2)
    # vx_rl_for_slip = sqrt(vx_rl**2 + eps_vx**2)
    # vx_rr_for_slip = sqrt(vx_rr**2 + eps_vx**2)

    ## Slip ratios
    slip_ratio_fl = 0.1 * state[9]  # (wheel_radius * state[9] - vx_fl) / vx_fl_for_slip
    slip_ratio_fr = 0.1 * state[10] # (wheel_radius * state[10] - vx_fr) / vx_fr_for_slip
    slip_ratio_rl = 0.1 * state[11] # (wheel_radius * state[11] - vx_rl) / vx_rl_for_slip
    slip_ratio_rr = 0.1 * state[12] # (wheel_radius * state[12] - vx_rr) / vx_rr_for_slip

    ## Aerodynamic forces 
    #total_downforce = 0.5 * air_density * aero_area * cda * state[3]**2
    #total_drag = 0.5 * air_density * aero_area * cla * state[3]**2
#
    ## Downforce distribution per wheel
    #d_force_fl = total_downforce * ((lr + cp_location) / L) * 0.5
    #d_force_fr = total_downforce * ((lr + cp_location) / L) * 0.5
    #d_force_rl = total_downforce * ((lf - cp_location) / L) * 0.5
    #d_force_rr = total_downforce * ((lf - cp_location) / L) * 0.5
#
    ## Weight transfer
    #longitudinal_weight_transfer = (h_cg * m * state[6]) / L
    #lateral_weight_transfer = (h_cg * m * state[7]) / (2 * s)

    # Normal load on each tire
    vertical_load_fl = m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fl - longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
    vertical_load_fr = m/4  #m * gravity_acceleration * lr / (L * 2) + d_force_fr - longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2
    vertical_load_rl = m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rl + longitudinal_weight_transfer / 2 - lateral_weight_transfer / 2
    vertical_load_rr = m/4  #m * gravity_acceleration * lf / (L * 2) + d_force_rr + longitudinal_weight_transfer / 2 + lateral_weight_transfer / 2

    ## Rolling resistance forces
    #rolling_resistance_fl = rolling_resistance_coefficient * vertical_load_fl * tanh(state[9])
    #rolling_resistance_fr = rolling_resistance_coefficient * vertical_load_fr * tanh(state[10])
    #rolling_resistance_rl = rolling_resistance_coefficient * vertical_load_rl * tanh(state[11])
    #rolling_resistance_rr = rolling_resistance_coefficient * vertical_load_rr * tanh(state[12])

    ## Tire forces
    fx_fl = tire_C * vertical_load_fl * slip_ratio_fl #(tire_D * vertical_load_fl * sin(tire_C * atan(tire_B * slip_ratio_fl - tire_E * (tire_B * slip_ratio_fl - atan(tire_B * slip_ratio_fl))))) - rolling_resistance_fl
    fy_fl = tire_C * vertical_load_fl * slip_angle_fl #(tire_lateral_D * vertical_load_fl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fl - tire_lateral_E * (tire_lateral_B * slip_angle_fl - atan(tire_lateral_B * slip_angle_fl)))))
    fx_fr = tire_C * vertical_load_fr * slip_ratio_fr #(tire_D * vertical_load_fr * sin(tire_C * atan(tire_B * slip_ratio_fr - tire_E * (tire_B * slip_ratio_fr - atan(tire_B * slip_ratio_fr))))) - rolling_resistance_fr
    fy_fr = tire_C * vertical_load_fr * slip_angle_fr #(tire_lateral_D * vertical_load_fr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_fr - tire_lateral_E * (tire_lateral_B * slip_angle_fr - atan(tire_lateral_B * slip_angle_fr)))))
    fx_rl = tire_C * vertical_load_rl * slip_ratio_rl #(tire_D * vertical_load_rl * sin(tire_C * atan(tire_B * slip_ratio_rl - tire_E * (tire_B * slip_ratio_rl - atan(tire_B * slip_ratio_rl))))) - rolling_resistance_rl
    fy_rl = tire_C * vertical_load_rl * slip_angle_rl #(tire_lateral_D * vertical_load_rl * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rl - tire_lateral_E * (tire_lateral_B * slip_angle_rl - atan(tire_lateral_B * slip_angle_rl)))))
    fx_rr = tire_C * vertical_load_rr * slip_ratio_rr #(tire_D * vertical_load_rr * sin(tire_C * atan(tire_B * slip_ratio_rr - tire_E * (tire_B * slip_ratio_rr - atan(tire_B * slip_ratio_rr))))) - rolling_resistance_rr
    fy_rr = tire_C * vertical_load_rr * slip_angle_rr #(tire_lateral_D * vertical_load_rr * sin(tire_lateral_C * atan(tire_lateral_B * slip_angle_rr - tire_lateral_E * (tire_lateral_B * slip_angle_rr - atan(tire_lateral_B * slip_angle_rr)))))

    ## Total forces
    ax_cog = (fx_fl + fx_fr + fx_rl + fx_rr)/m # state[5] * state[4] + (fx_fl * cos(sa_fl) - fy_fl * sin(sa_fl) + fx_fr * cos(sa_fr) - fy_fr * sin(sa_fr) + fx_rl * cos(sa_rl) - fy_rl * sin(sa_rl) + fx_rr * cos(sa_rr) - fy_rr * sin(sa_rr)) / m # - total_drag) / m
    ay_cog = (fy_fl + fy_fr + fy_rl + fy_rr)/m # - state[5] * state[3] +(fx_fl * sin(sa_fl) + fy_fl * cos(sa_fl) + fx_fr * sin(sa_fr) + fy_fr * cos(sa_fr) + fx_rl * sin(sa_rl) + fy_rl * cos(sa_rl) + fx_rr * sin(sa_rr) + fy_rr * cos(sa_rr)) / m
    yaw_rate_dot = (fy_fl + fy_fr + fy_rl + fy_rr) / (Izz + m) # (s * (fy_fr * cos(sa_fr) + fx_fr * sin(sa_fr)) - s * (fy_fl * cos(sa_fl) + fx_fl * sin(sa_fl)) + lr * (fx_fr * cos(sa_fr) - fy_fr * sin(sa_fr)) + lr * (fx_fl * cos(sa_fl) - fy_fl * sin(sa_fl)) - lr * (fx_rr * cos(sa_rr) - fy_rr * sin(sa_rr)) - lr * (fx_rl * cos(sa_rl) - fy_rl * sin(sa_rl))) / Izz

    ## Max torque at wheels
    #max_torque_fl = 0
    #max_torque_fr = 0
    #max_torque_rl = fmin(max_motor_power / (sqrt(state[11]**2) + 1e-3), max_motor_torque)
    #max_torque_rr = fmin(max_motor_power / (sqrt(state[12]**2) + 1e-3), max_motor_torque)
#
    ## Demanded torque at wheels
    demanded_torque_fl = 0 # max_torque_fl * 0
    demanded_torque_fr = 0 # max_torque_fr * 0
    demanded_torque_rl = max_motor_torque * controls[0] # fmin(max_motor_torque * controls[0], max_torque_rl)
    demanded_torque_rr = max_motor_torque * controls[1] # fmin(max_motor_torque * controls[1], max_torque_rr)
#
    ## Angular acceleration at wheels
    alpha_fl = (demanded_torque_fl - wheel_radius * fx_fl) / wheel_rotational_inertia
    alpha_fr = (demanded_torque_fr - wheel_radius * fx_fr) / wheel_rotational_inertia
    alpha_rl = (demanded_torque_rl - wheel_radius * fx_rl) / wheel_rotational_inertia
    alpha_rr = (demanded_torque_rr - wheel_radius * fx_rr) / wheel_rotational_inertia

    # Steering rate
    steering_dot = (controls[2] - state[8]) / steering_motor_tau

    # Dynamics (Explicit ODE)
    f_expl = vertcat(
        state[3] * cos(state[2]) - state[4] * sin(state[2]),
        state[3] * sin(state[2]) + state[4] * cos(state[2]),
        state[5],
        ax_cog - state[5] * state[4],
        ay_cog + state[5] * state[3],
        yaw_rate_dot,
        (ax_cog - state[6]) / 0.05,
        (ay_cog - state[7]) / 0.05,
        steering_dot,
        alpha_fl,
        alpha_fr,
        alpha_rl,
        alpha_rr
    )

    model.f_expl_expr = f_expl
    model.x = state
    model.u = controls

    constraint_expr = controls[0] - controls[1]  # difference between controls
    model.con_h_expr = constraint_expr

    return model


def setup_cost_function(ocp: AcadosOcp):
    """
    Sets up the cost function to track the closest point on the path.
    Car position is given by the first 2 states (x, y).
    Path is passed via p_global as 50 points with 3 values each.
    """

    # Extract symbolic variables
    x_sym = ocp.model.x
    p_global_sym = ocp.model.p_global

    # Car position (first 2 states)
    car_pos = x_sym[:2]
    #car_orientation = x_sym[2]
    car_speed = x_sym[3]

    # Reshape path from p_global
    path_points = p_global_sym.reshape((4, path_size))
    p_x = path_points[0, :]
    p_y = path_points[1, :]
    #p_velocity = path_points[2, :]
    #p_orientation = path_points[3, :]

    distances = (p_x - car_pos[0]) ** 2 + (p_y - car_pos[1]) ** 2

    # Find minimum distance using symbolic fmin (no Python if/else with symbolic expressions)
    min_distance = distances[0]
    for i in range(1, path_size):
        min_distance = fmin(min_distance, distances[i])

    #eps = 0.5
    #weights = exp(-distances / eps)
    #W = sum(weights)
#
    #velocity_ref = sum(weights * p_velocity) / W
    #velocity_error = car_speed - velocity_ref
#
    #orientation_ref = sum(weights * p_orientation) / W
    #orientation_error = car_orientation - orientation_ref

    # u_sym = ocp.model.u
    #control_penalty = (
    #    u_rl_weight * u_sym[0] ** 2
    #    + u_rr_weight * u_sym[1] ** 2
    #    + u_steer_weight * u_sym[2] ** 2
    #)

    cost_expr = velocity_weight * ((car_speed - 5)**2) # (distance_weight * min_distance) + 
        #+ velocity_weight * velocity_error**2
        #+ orientation_weight * orientation_error**2
        #+ control_penalty)

    terminal_cost_expr = velocity_weight * ((car_speed - 5)**2) # (distance_weight * min_distance) +
    #    + velocity_weight * velocity_error**2
    #    + orientation_weight * orientation_error**2
    #)

    # Set up as external cost (more flexible)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model.cost_expr_ext_cost = cost_expr
    ocp.model.cost_expr_ext_cost_e = terminal_cost_expr


def create_ocp_solver(gen_base_dir="./build/acados"):
    c_code_dir = os.path.abspath("./src/control/include/solver/acados/c_generated_code")
    json_path = os.path.abspath(os.path.join(gen_base_dir, "acados_ocp_mpc.json"))

    if os.path.exists(c_code_dir):
        shutil.rmtree(c_code_dir)
    if os.path.exists(json_path):
        os.remove(json_path)

    os.makedirs(os.path.dirname(c_code_dir), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)

    ocp = AcadosOcp()
    ocp.model = export_mpc_model()
    ocp.code_export_directory = c_code_dir

    # ocp.solver_options.N_horizon = 10
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.with_batch_functionality = True

    ocp.solver_options.N_horizon = 30 # Try 30 to get 0.05s steps
    ocp.solver_options.tf = 1.5 
    
    # 1. Use full SQP to allow the solver to find a feasible path from the guess
    #ocp.solver_options.nlp_solver_type = "SQP"
    # ocp.solver_options.nlp_solver_max_iter = 100 
    
    # 2. Use Implicit integrator for stiff Pacejka tire dynamics
    # ocp.solver_options.integrator_type = "IRK"
    # ocp.solver_options.sim_method_num_stages = 4
    # ocp.solver_options.sim_method_num_steps = 3 # Sub-steps for better accuracy
    
    #ocp.solver_options.print_level = 0
    #ocp.solver_options.regularize_method = "PROJECT"
    #ocp.solver_options.reg_epsilon = 1e-1
    #ocp.solver_options.levenberg_marquardt = 1e-1


    # Initial state constraint (required for set_state logic)
    ocp.constraints.x0 = np.zeros(13)

    # Initialize p_global with dummy path data (50 points x 4 values)
    # Each point has [x, y, velocity, orientation]
    dummy_path = np.zeros(path_size * path_point_size)
    ocp.p_global_values = dummy_path

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
