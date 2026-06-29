import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, sqrt, atan, atan2, tan, if_else, fabs, tanh, fmin, fabs
from ament_index_python.packages import get_package_prefix
import yaml

lr = 0.804  # Distance from the center of mass to the rear axle
lf = 0.726  # Distance from the center of mass to the front axle
L = lr + lf

rolling_resistance_coefficient = 0.015

gravity_acceleration = 9.81

steering_motor_tau = 0.25

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
    Load MPC parameters for the adapter selected in the global config YAML file.
    """
    global_config_path = get_config_yaml_path("common_lib", "global", "global_config")
    
    with open(global_config_path, 'r') as f:
        global_config = yaml.safe_load(f)

    adapter = global_config["global"]["adapter"]
    control_config_path = get_config_yaml_path("common_lib", "control", adapter)

    with open(control_config_path, 'r') as f:
        control_config = yaml.safe_load(f)
    
    mpc_horizon_time = control_config["control"]["mpc_prediction_horizon_seconds"]
    mpc_horizon_steps = control_config["control"]["mpc_prediction_horizon_steps"]
    max_steering_command_derivative = control_config["control"]["mpczinho_max_steering_command_derivative"]
    command_time_interval_seconds = control_config["control"]["command_time_interval"] / 1000.0
    
    return mpc_horizon_time, mpc_horizon_steps, max_steering_command_derivative, command_time_interval_seconds

def export_mpc_model(command_time_interval_seconds: float) -> AcadosModel:
    model = AcadosModel()
    model.name = "mpczinho"
    p = SX.sym("p", 5) # [x_ref, y_ref, v, theta_ref, previous_steering_command]

    x = SX.sym("x", 4) # [x_position, y_position, yaw, steering_angle]
    xdot = SX.sym("xdot", 4)
    u = SX.sym("u", 1)

    # dynamics
    f_expl = vertcat(
        p[2] * cos(x[2]),
        p[2] * sin(x[2]),
        p[2] * tan(u[0]) / L,
        (u[0] - x[3]) / steering_motor_tau  # Assuming steering angle dynamics
    )

    model.p = p
    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    steering_command_derivative = (u[0] - p[4]) / command_time_interval_seconds
    model.con_h_expr = steering_command_derivative
    model.con_h_expr_0 = steering_command_derivative
    return model

def setup_cost_function(ocp: AcadosOcp):
    """
    Sets up the cost function to track a SPECIFIC reference point 'p' per stage.
    """
    # 1. Extract symbolic variables
    x = ocp.model.x
    u = ocp.model.u
    p = ocp.model.p 

    # 2. Configure Cost Type
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # 3. Define the Residuals
    psi = x[2]
    ref_theta = p[3]
    theta_cost_term = sin(psi - ref_theta)

    cost_expression = vertcat(
        x[0] - p[0],      # X Position Error
        x[1] - p[1],      # Y Position Error
        theta_cost_term,  # Orientation Error
        u[0],             # Penalty on Steering
        u[0] - p[4]       # Penalty on steering command change
    )

    # Terminal Residual (No controls at the last step)
    cost_expression_e = vertcat(
        x[0] - p[0],
        x[1] - p[1],
        theta_cost_term
    )

    ocp.model.cost_y_expr = cost_expression
    ocp.model.cost_y_expr_e = cost_expression_e

    # 4. Define Weight Matrices (W)
    weights = np.array([2.0, 2.0, 1.0, 8.0, 5.0])
    
    # Terminal weights
    weights_e = np.array([2.0, 2.0, 1.0])

    ocp.cost.W = np.diag(weights)
    ocp.cost.W_e = np.diag(weights_e)

    # 5. Set Numeric References to Zero
    # ny = 5 (4 states + 1 control)
    ocp.cost.yref = np.zeros(5)
    ocp.cost.yref_e = np.zeros(3)

    # IMPORTANT: Update Dimensions
    ocp.dims.ny = 5
    ocp.dims.ny_e = 3


def create_ocp_solver(gen_base_dir: str = "./build/control/control/mpczinho/acados", acados_dir: str | None = None, acados_lib_dir: str | None = None):
    # gen_base_dir should point to a folder that will contain the generated
    # acados artifacts. We place the C sources under <gen_base_dir>/c_generated_code
    # and the json model at <gen_base_dir>/acados_ocp_mpc.jsons
    c_code_dir = os.path.abspath(os.path.join(gen_base_dir, "c_generated_code"))
    json_path = os.path.abspath(os.path.join(gen_base_dir, "acados_ocp_mpc.json"))

    if os.path.exists(c_code_dir):
        shutil.rmtree(c_code_dir)
    if os.path.exists(json_path):
        os.remove(json_path)

    os.makedirs(os.path.dirname(c_code_dir), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)

    prediction_horizon_seconds, prediction_horizon_steps, max_steering_command_derivative, command_time_interval_seconds = load_mpc_parameters()

    if acados_dir is not None:
        acados_dir = os.path.abspath(acados_dir)
    if acados_lib_dir is not None:
        acados_lib_dir = os.path.abspath(acados_lib_dir)

    if acados_dir is not None:
        ocp = AcadosOcp(acados_path=acados_dir, acados_lib_path=acados_lib_dir)
    else:
        ocp = AcadosOcp()
    ocp.model = export_mpc_model(command_time_interval_seconds)
    ocp.code_export_directory = c_code_dir

    ocp.solver_options.N_horizon = prediction_horizon_steps
    ocp.solver_options.tf = prediction_horizon_seconds
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.levenberg_marquardt = 1e-2
    ocp.solver_options.with_batch_functionality = False
    ocp.solver_options.ext_fun_compile_flags = "-O3 -march=native -ffast-math"
    ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.with_batch_functionality = True 
    ocp.solver_options.num_threads_in_batch_ext_fun = 4

    ocp.solver_options.nlp_solver_tol_stat = 1e-2
    ocp.solver_options.nlp_solver_tol_eq = 1e-2

    # Initial state constraint (required for set_state logic)
    ocp.constraints.x0 = np.zeros(4)
    ocp.parameter_values = np.zeros(5)

    setup_cost_function(ocp)

    # Lower bounds for controls
    u_min = np.array([-0.335])  # adjust these values as needed

    # Upper bounds for controls
    u_max = np.array([0.335])  # adjust these values as needed

    ocp.constraints.lbu = u_min  # lower bound on u
    ocp.constraints.ubu = u_max  # upper bound on u
    ocp.constraints.idxbu = np.array([0])  # which control inputs have bounds

    ocp.constraints.lh = np.array([-max_steering_command_derivative])
    ocp.constraints.uh = np.array([max_steering_command_derivative])
    ocp.dims.nh = 1

    ocp.constraints.lh_0 = np.array([-max_steering_command_derivative])
    ocp.constraints.uh_0 = np.array([max_steering_command_derivative])
    ocp.dims.nh_0 = 1

    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        return solver
    except Exception as e:
        print(f"FAIL: {e}")
        return None

if __name__ == "__main__":
    # Allow specifying an output directory so multiple MPCs can coexist
    import argparse

    parser = argparse.ArgumentParser(description="Generate acados C code for MPC")
    parser.add_argument("--out-dir", dest="out_dir", default="./build/control/control/mpczinho/acados",
                        help="Base output directory for generated files (contains c_generated_code/ and json)")
    parser.add_argument("--acados-dir", dest="acados_dir", default="./build/acados-install",
                        help="Path to acados install root (contains include/ and lib/)")
    parser.add_argument("--acados-lib-dir", dest="acados_lib_dir", default="./build/acados-install/lib",
                        help="Path to acados libraries (defaults to <acados-dir>/lib)")
    args = parser.parse_args()

    create_ocp_solver(args.out_dir, args.acados_dir, args.acados_lib_dir)
