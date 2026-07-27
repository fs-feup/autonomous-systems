import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, sqrt, atan, atan2, tan, if_else, fabs, tanh, fmin, fabs
from ament_index_python.packages import get_package_prefix
import yaml

# Geometry mirrors config/car/02.yaml: wheel_base 1.53, cg_2_rear_axis 0.706.
# The simulator uses lr = cg_2_rear_axis and lf = wheel_base - lr (see
# FSFEUP02::get_state_derivative), so the controller must use the same split.
lr = 0.706  # Distance from the center of mass to the rear axle
lf = 0.824  # Distance from the center of mass to the front axle
L = lr + lf

rolling_resistance_coefficient = 0.015

gravity_acceleration = 9.81

# First-order steering actuator, matching the simulator's FirstOrderSteeringMotor
# with time_constant from config/car/steering_motor_model/02_steering_motor.yaml.
steering_motor_tau = 0.112

max_steering_angle = 0.335  # config/car/steering_model/02_steering.yaml

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

def get_active_adapter() -> str:
    """
    Read the active adapter from the global config so the generated solver is
    dimensioned from the same control config the node loads at runtime.
    """
    with open(get_config_yaml_path("common_lib", "global", "global_config"), 'r') as f:
        return yaml.safe_load(f)["global"]["adapter"]


def load_mpc_parameters():
    """
    Load MPC horizon time and steps from the active adapter's control config.
    """
    control_config_path = get_config_yaml_path("common_lib", "control", get_active_adapter())

    with open(control_config_path, 'r') as f:
        control_config = yaml.safe_load(f)

    control = control_config["control"]
    mpc_horizon_time = control.get("lateral_mpc_prediction_horizon_seconds",
                                   control["mpc_prediction_horizon_seconds"])
    mpc_horizon_steps = control.get("lateral_mpc_prediction_horizon_steps",
                                    control["mpc_prediction_horizon_steps"])

    return mpc_horizon_time, mpc_horizon_steps

def export_mpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "mpczinho"
    p = SX.sym("p", 4) # [x_ref, y_ref, v, theta_ref]

    x = SX.sym("x", 4) # [x_position, y_position, yaw, steering_angle]
    xdot = SX.sym("xdot", 4)
    u = SX.sym("u", 1) # [steering_angle_command]

    # Kinematic bicycle written at the CENTRE OF GRAVITY, because the pose the
    # controller receives (/state_estimation/vehicle_pose) is the CG pose, not the
    # rear axle. Using the rear-axle form with a CG pose biases the predicted
    # trajectory by lr*psi_dot laterally, which shows up as a steady corner offset.
    beta = atan(lr * tan(x[3]) / L)

    # The yaw rate is driven by the ACTUAL steering angle state x[3], not by the
    # command u[0]. The command only feeds the first-order steering actuator, so
    # the model reproduces the simulator's steering lag instead of assuming the
    # wheels reach the commanded angle instantly.
    f_expl = vertcat(
        p[2] * cos(x[2] + beta),
        p[2] * sin(x[2] + beta),
        p[2] * cos(beta) * tan(x[3]) / L,
        (u[0] - x[3]) / steering_motor_tau
    )

    model.p = p
    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
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
        x[3],             # Penalty on Steering
        u[0] - x[3]       # Penalty on Steering Rate (u - delta = tau * delta_dot)
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
    # [x_err, y_err, heading_err, steering_magnitude, steering_rate]
    weights = np.array([9.0, 9.0, 6.0, 0.3, 2.0])

    # Terminal weights
    weights_e = np.array([9.0, 9.0, 6.0])

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

    prediction_horizon_seconds, prediction_horizon_steps = load_mpc_parameters()

    if acados_dir is not None:
        acados_dir = os.path.abspath(acados_dir)
    if acados_lib_dir is not None:
        acados_lib_dir = os.path.abspath(acados_lib_dir)

    if acados_dir is not None:
        ocp = AcadosOcp(acados_path=acados_dir, acados_lib_path=acados_lib_dir)
    else:
        ocp = AcadosOcp()
    ocp.model = export_mpc_model()
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
    ocp.parameter_values = np.zeros(4)

    setup_cost_function(ocp)

    # The only hard actuator limit the simulator enforces is the steering angle
    # range (AckermanSteering clamps to +-0.335 rad). There is no rate limit in
    # FirstOrderSteeringMotor, so imposing one here would be a model mismatch;
    # steering rate is shaped by the cost instead.
    ocp.constraints.lbu = np.array([-max_steering_angle])
    ocp.constraints.ubu = np.array([max_steering_angle])
    ocp.constraints.idxbu = np.array([0])

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
