import numpy as np
import math
from scipy.interpolate import interp1d
import cvxpy as opt

from .config import Params

np.seterr(divide="ignore", invalid="ignore")


P = Params()

def compute_path_from_wp(path, step=0.1):
    """
    Computes a reference path given a set of waypoints
    """
    start_xp = path[:, 0]
    start_yp = path[:, 1]

    final_xp = []
    final_yp = []

    delta = step  # [m]

    for idx in range(len(path) - 1):
        # distance between 2 consecutive points
        section_len = np.sum(
            np.sqrt(
                np.power(np.diff(start_xp[idx : idx + 2]), 2)
                + np.power(np.diff(start_yp[idx : idx + 2]), 2)
            )
        )

        # generated divisions to generate new points 
        interp_range = np.linspace(0, 1, np.floor(section_len / delta).astype(int))

        # linear function between 2 consecutive positions
        fx = interp1d(np.linspace(0, 1, 2), start_xp[idx : idx + 2], kind=1)
        fy = interp1d(np.linspace(0, 1, 2), start_yp[idx : idx + 2], kind=1)

        # watch out to duplicate points! - do not add first index
        final_xp = np.append(final_xp, fx(interp_range)[1:])
        final_yp = np.append(final_yp, fy(interp_range)[1:])
    # add (0, 0) point to first index
    dx = np.append(0, np.diff(final_xp))
    dy = np.append(0, np.diff(final_yp))
    theta = np.arctan2(dy, dx)
    return np.vstack((final_xp, final_yp, theta)).T


def get_nn_idx(state, path, old_nn_idx):
    """
    Computes the index of the waypoint closest to vehicle
    """

    search_window = 100
    
    # window in intersection [0, len(path)] and [index - 100, index + 100]
    windowed_path = path[max(old_nn_idx - search_window, 0): 
                            min(old_nn_idx + search_window, path.shape[0]), :]
    
    # distances between state and path points
    dx = state[0] - windowed_path[:, 0]
    dy = state[1] - windowed_path[:, 1]
    dist = np.hypot(dx, dy)

    # get index of min dist 
    nn_idx = np.argmin(dist) + max(old_nn_idx - search_window, 0)

    try:
        # enforce the closest point to be the one in front of vehicle
        v = [
            path[nn_idx + 1, 0] - path[nn_idx, 0],
            path[nn_idx + 1, 1] - path[nn_idx, 1],
        ]
        v /= np.linalg.norm(v)
        d = [path[nn_idx, 0] - state[0], path[nn_idx, 1] - state[1]]
        if np.dot(d, v) > 0:
            target_idx = nn_idx
        else:
            target_idx = nn_idx + 1
    except IndexError:
        target_idx = nn_idx
    return target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def get_ref_trajectory(state, path, target_v, old_ind=0):
    """
    For each step in the time horizon
    modified reference in car frame
    """

    # initialize variables
    xref = np.zeros((P.state_len, P.prediction_horizon + 1))
    uref = np.zeros((P.command_len, P.prediction_horizon + 1))

    path_len = path.shape[0]

    # get next point in path (relative to car)
    ind = get_nn_idx(state, path, old_ind)

    # current distance to path
    dx = path[ind, 0] - state[0]
    dy = path[ind, 1] - state[1]

    # first position references (using local coordinate system)
    xref[0, 0] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])  # X
    xref[1, 0] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])  # Y
    xref[2, 0] = target_v  # V
    xref[3, 0] = normalize_angle(path[ind, 2] - state[3])  # Theta

    travel = 0.0  # distance traveled based on reference velocity

    # iterate over timesteps in the optimization
    for i in range(1, P.prediction_horizon + 1):
        # update distance traveled
        travel += abs(target_v) * P.DT
        dind = int(round(travel / P.path_tick))
        
        # if path ends, no calculation performed
        if (ind + dind) < path_len:
            # update expected position (relative to car)
            dx = path[ind + dind, 0] - state[0]
            dy = path[ind + dind, 1] - state[1]

            # position references (local ref)
            xref[0, i] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])
            xref[1, i] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])
            xref[2, i] = target_v
            xref[3, i] = normalize_angle(path[ind + dind, 2] - state[3])

        else:
            dx = path[path_len - 1, 0] - state[0]
            dy = path[path_len - 1, 1] - state[1]
            xref[0, i] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])
            xref[1, i] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])
            xref[2, i] = 0.0
            xref[3, i] = normalize_angle(path[path_len - 1, 2] - state[3])
    return xref, uref, ind


def get_linear_model_matrices(x_bar, u_bar):
    """
    Computes the LTI approximated state space model x' = A'x + B'u + C
    """

    # state variables (x)
    x = x_bar[0]
    y = x_bar[1]
    v = x_bar[2]
    theta = x_bar[3]

    # command variables (u)
    a = u_bar[0]
    delta = u_bar[1]

    ct = np.cos(theta)
    st = np.sin(theta)
    cd = np.cos(delta)
    td = np.tan(delta)

    A = np.zeros((P.state_len, P.state_len))
    A[0, 2] = ct
    A[0, 3] = -v * st
    A[1, 2] = st
    A[1, 3] = v * ct
    A[3, 2] = v * td / P.L
    A_lin = np.eye(P.state_len) + P.DT * A

    B = np.zeros((P.state_len, P.command_len))
    B[2, 0] = 1
    B[3, 1] = v / (P.L * cd**2)
    B_lin = P.DT * B

    f_xu = np.array([v * ct, v * st, a, v * td / P.L]).reshape(P.state_len, 1)

    C_lin = (
        P.DT
        * (
            f_xu - np.dot(A, x_bar.reshape(P.state_len, 1)) - np.dot(B, u_bar.reshape(P.command_len, 1))
        ).flatten()
    )

    return np.round(A_lin,6), np.round(B_lin,6), np.round(C_lin,6)


def optimize(A, B, C, initial_state, x_ref, u_ref, verbose=False):
        """
        Optimisation problem defined for the linearised model,
        :param A: model matrix A'
        :param B: model matrix B'
        :param C: model matrix C'
        :param initial_state: car's initial state
        :param verbose: verbose for optimization problem
        :return: optimized states and actions
        """

        assert len(initial_state) == P.state_len

        # Create variables
        x = opt.Variable((P.state_len, P.prediction_horizon + 1), name="states")
        u = opt.Variable((P.command_len, P.prediction_horizon), name="actions")

        # Loop through the entire time_horizon and append costs
        cost_function = []

        for t in range(P.prediction_horizon):

            _cost = \
                opt.quad_form(x_ref[:, t + 1] - x[:, t + 1], P.state_cost) + \
                opt.quad_form(u_ref[:, t] - u[:, t], P.command_cost)

            _constraints = [
                x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C,
                u[0, t] >= -P.MAX_ACC,
                u[0, t] <= P.MAX_ACC,
                u[1, t] >= -P.MAX_STEER,
                u[1, t] <= P.MAX_STEER,
            ]

            # Actuation rate of change
            if t < (P.prediction_horizon - 1):
                _cost += opt.quad_form(u[:, t + 1] - u[:, t], P.command_cost * 1)

                _constraints += [opt.abs(u[0, t + 1] - u[0, t]) / P.DT <= P.MAX_D_ACC]
                _constraints += [opt.abs(u[1, t + 1] - u[1, t]) / P.DT <= P.MAX_D_STEER]

            if t == 0:
                _constraints += [x[:, 0] == initial_state]

            cost_function.append(
                opt.Problem(opt.Minimize(_cost), constraints=_constraints)
            )

        # Add final cost
        problem = sum(cost_function)

        # Minimize Problem
        problem.solve(verbose=verbose, solver=opt.OSQP)

        return x, u


def wheel_rpm_2_wheel_vel(rpm):
    return rpm * 0.5 * math.pi / 60


def wheels_vel_2_vehicle_vel(fl_vel, fr_vel, rl_vel, rr_vel, steering_angle):
    if not steering_angle or (steering_angle <= 0.05 and steering_angle >= -0.05):
        return (wheel_rpm_2_wheel_vel(fl_vel) +
            wheel_rpm_2_wheel_vel(fr_vel) +
            wheel_rpm_2_wheel_vel(rl_vel) +
            wheel_rpm_2_wheel_vel(rr_vel)) / 4
    
    if not steering_angle:
        return wheel_rpm_2_wheel_vel(rl_vel)
    elif steering_angle > 0:
        wheel_velocity = wheel_rpm_2_wheel_vel(rl_vel)
        rear_axis_center_rotation_radius = P.L / np.tan(steering_angle)
        w = wheel_velocity / (rear_axis_center_rotation_radius - (P.car_width / 2))
        r = np.sqrt(P.Lc**2 + rear_axis_center_rotation_radius**2)
        return w * r
    else:
        wheel_velocity = wheel_rpm_2_wheel_vel(rr_vel)
        rear_axis_center_rotation_radius = P.L / np.tan(steering_angle)
        w = wheel_velocity / (rear_axis_center_rotation_radius + (P.car_width / 2))
        r = np.sqrt(P.Lc**2 + rear_axis_center_rotation_radius**2)
        return w * r
