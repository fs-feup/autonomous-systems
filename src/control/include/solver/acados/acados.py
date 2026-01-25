import sys
import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, horzcat, sqrt, fmin, tan, if_else

path_size = 50


def export_mpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "mpc"
    model.p_global = SX.sym(
        "p_global", path_size * 3
    )  # Path: 100 points x 3 values each

    # States (Dimension 13)
    x = SX.sym("x", 13)

    # Controls (Dimension 3)
    u = SX.sym("u", 3)

    # Dynamics (Explicit ODE)
    # First 3 state derivatives equal the controls
    # Remaining 10 state derivatives are zero
    f_expl = vertcat(
        x[3] * cos(x[2]),  # - x[4] * sin(x[2]),
        x[3] * sin(x[2]),  # + x[4] * cos(x[2]),
        tan(u[2]) * x[3] / 1.5,
        (u[0] + u[1]) * 80,
        SX.zeros(9, 1),
    )

    # model.f_impl_expr = x - x  # Dummy for implicit compatibility
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u

    constraint_expr = u[0] - u[1]  # difference between controls
    model.con_h_expr = constraint_expr

    return model


def setup_cost_function(ocp: AcadosOcp):
    """
    Sets up the cost function to track the closest point on the path.
    Car position is given by the first 2 states (x, y).
    Path is passed via p_global as 100 points with 3 values each.
    """

    # Extract symbolic variables
    x_sym = ocp.model.x
    p_global_sym = ocp.model.p_global

    # Car position (first 2 states)
    car_pos = x_sym[:2]
    car_speed = x_sym[3]

    # Reshape path from p_global
    path_points = p_global_sym.reshape((3, path_size))
    px = path_points[0, :]
    py = path_points[1, :]
    pv = path_points[2, :]

    distances = (px - car_pos[0]) ** 2 + (py - car_pos[1]) ** 2

    # Find minimum distance using symbolic fmin (no Python if/else with symbolic expressions)
    min_distance = distances[0]
    # index = 0
    for i in range(1, path_size):
        min_distance = fmin(min_distance, distances[i])
        # if_else(min_distance == distances[i], index := i, index := index)

    speed_error = car_speed - 5  #  path_points[index, 2]

    # Define the cost expression as the squared distance to closest point
    # We use only the distance penalty without velocity matching since we can't
    # symbolically index into path_points with a symbolic index
    cost_expr = min_distance + speed_error**2

    # Set up as external cost (more flexible)
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model.cost_expr_ext_cost = cost_expr
    ocp.model.cost_expr_ext_cost_e = cost_expr


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

    ocp.dims.N = 50
    ocp.solver_options.tf = 1.5
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.with_batch_functionality = True

    # Initial state constraint (required for set_state logic)
    ocp.constraints.x0 = np.zeros(13)

    # Initialize p_global with dummy path data (100 points x 3 values)
    # Each point has [x, y, velocity]
    dummy_path = np.zeros(path_size * 3)
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
