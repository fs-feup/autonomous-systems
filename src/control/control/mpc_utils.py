import numpy as np
import math
from scipy.interpolate import interp1d

from .pid_utils import get_closest_point
from .config import Params

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
        section_len = np.sum(
            np.sqrt(
                np.power(np.diff(start_xp[idx : idx + 2]), 2)
                + np.power(np.diff(start_yp[idx : idx + 2]), 2)
            )
        )

        interp_range = np.linspace(0, 1, np.floor(section_len / delta).astype(int))

        fx = interp1d(np.linspace(0, 1, 2), start_xp[idx : idx + 2], kind=1)
        fy = interp1d(np.linspace(0, 1, 2), start_yp[idx : idx + 2], kind=1)

        # watch out to duplicate points!
        final_xp = np.append(final_xp, fx(interp_range)[1:])
        final_yp = np.append(final_yp, fy(interp_range)[1:])
    dx = np.append(0, np.diff(final_xp))
    dy = np.append(0, np.diff(final_yp))
    theta = np.arctan2(dy, dx)
    return np.vstack((final_xp, final_yp, theta)).T


def get_nn_idx(state, path, old_nn_idx):
    """
    Computes the index of the waypoint closest to vehicle
    """

    search_window = 100
    
    windowed_path = path[max(old_nn_idx - search_window, 0): 
                            min(old_nn_idx + search_window, path.shape[0]), :]
    
    dx = state[0] - windowed_path[:, 0]
    dy = state[1] - windowed_path[:, 1]

    dist = np.hypot(dx, dy)
    
    nn_idx = np.argmin(dist) + max(old_nn_idx - search_window, 0)
    
    try:
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


def get_ref_trajectory(state, path, target_v, dl=0.1, old_ind=0):
    """
    For each step in the time horizon
    modified reference in car frame
    """

    # initialize variables
    xref = np.zeros((P.N, P.T + 1))
    uref = np.zeros((P.M, P.T + 1))

    uref[1, :] = np.ones((1,P.T +1))*target_v #speed profile

    path_len = path.shape[0]

    ind = get_closest_point(state, path, old_ind, search_window=100)

    dx = path[ind, 0] - state[0]
    dy = path[ind, 1] - state[1]

    # first position references
    xref[0, 0] = dx * np.cos(-state[2]) - dy * np.sin(-state[2])  # X
    xref[1, 0] = dy * np.cos(-state[2]) + dx * np.sin(-state[2])  # Y
    xref[2, 0] = normalize_angle(path[ind, 2] - state[2])  # Theta

    travel = 0.0  # distance traveled based on reference velocity

    # iterate over timesteps in the optimization
    for i in range(1, P.T + 1):
        # update distance traveled
        travel += abs(target_v) * P.DT
        dind = int(round(travel / dl))
        
        if (ind + dind) < path_len:
            # update expected position
            dx = path[ind + dind, 0] - state[0]
            dy = path[ind + dind, 1] - state[1]

            # position references
            xref[0, i] = dx * np.cos(-state[2]) - dy * np.sin(-state[2])
            xref[1, i] = dy * np.cos(-state[2]) + dx * np.sin(-state[2])
            xref[2, i] = normalize_angle(path[ind + dind, 2] - state[2])
        else:
            dx = path[path_len - 1, 0] - state[0]
            dy = path[path_len - 1, 1] - state[1]
            xref[0, i] = dx * np.cos(-state[2]) - dy * np.sin(-state[2])
            xref[1, i] = dy * np.cos(-state[2]) + dx * np.sin(-state[2])
            xref[2, i] = normalize_angle(path[path_len - 1, 2] - state[2])

            # final speed reference
            uref[1, i] = 0

    return xref, uref, ind


def get_linear_model_matrices(x_bar, u_bar):
    """
    Computes the LTI approximated state space model x' = Ax + Bu + C
    """

    x_bar[0]
    x_bar[1]
    theta = x_bar[2]

    v = u_bar[0]
    delta = u_bar[1]

    ct = np.cos(theta)
    st = np.sin(theta)

    cd = np.cos(delta)
    sd = np.sin(delta)

    L_ratio = P.Lc / P.L

    A = np.zeros((P.N, P.N))
    A[0, 2] = -v * (st*cd + L_ratio*ct*sd)
    A[1, 2] = v * (ct*cd - L_ratio*st*sd)

    A_lin = np.eye(P.N) + P.DT * A

    B = np.zeros((P.N, P.M))
    B[0, 0] = ct*cd - L_ratio*st*sd
    B[1, 0] = L_ratio*ct*sd + st*cd
    B[2, 0] = sd / P.L
    
    B[0, 1] = - v * (ct*sd + L_ratio * st*cd) 
    B[1, 1] = v * (L_ratio*ct*sd + st*cd) 
    B[2, 1] = v * cd / P.L

    B_lin = P.DT * B

    f_xu = np.array(
        [
            v * (ct*cd - L_ratio*st*sd), 
            v * (L_ratio*ct*sd + st*cd), 
            v * sd / P.L
        ]
    ).reshape(P.N, 1)

    C_lin = (
        P.DT
        * (
            f_xu - np.dot(A, x_bar.reshape(P.N, 1)) - np.dot(B, u_bar.reshape(P.M, 1))
        ).flatten()
    )

    return np.round(A_lin,6), np.round(B_lin,6), np.round(C_lin,6)


def wheel_rpm_2_wheel_vel(rpm):
    return rpm * 0.5 * math.pi / 60


def wheels_vel_2_vehicle_vel(fl_vel, fr_vel, rl_vel, rr_vel, steering_angle):
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
