import numpy as np
from scipy.interpolate import interp1d
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
    return np.vstack((final_xp, final_yp, theta))


def get_nn_idx(state, path, old_nn_idx):
    """
    Computes the index of the waypoint closest to vehicle
    """
    search_window = 100
    
    windowed_path = path[:, max(old_nn_idx - search_window, 0): 
                            min(old_nn_idx + search_window, path.shape[1])]
    
    dx = state[0] - windowed_path[0, :]
    dy = state[1] - windowed_path[1, :]

    dist = np.hypot(dx, dy)
    
    nn_idx = np.argmin(dist) + max(old_nn_idx - search_window, 0)
    
    try:
        v = [
            path[0, nn_idx + 1] - path[0, nn_idx],
            path[1, nn_idx + 1] - path[1, nn_idx],
        ]
        v /= np.linalg.norm(v)
        d = [path[0, nn_idx] - state[0], path[1, nn_idx] - state[1]]
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
    dref = np.zeros((1, P.T + 1))
    # sp = np.ones((1,T +1))*target_v #speed profile

    path_len = path.shape[1]

    ind = get_nn_idx(state, path, old_ind)
    dx = path[0, ind] - state[0]
    dy = path[1, ind] - state[1]

    # first position references
    xref[0, 0] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])  # X
    xref[1, 0] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])  # Y
    xref[2, 0] = target_v  # V
    xref[3, 0] = normalize_angle(path[2, ind] - state[3])  # Theta

    # first steering reference
    dref[0, 0] = 0.0  # Steer operational point should be 0

    travel = 0.0  # distance traveled based on reference velocity

    # iterate over timesteps in the optimization
    for i in range(1, P.T + 1):
        # update distance traveled
        travel += abs(target_v) * P.DT
        dind = int(round(travel / dl))
        
        if (ind + dind) < path_len:
            # update expected position
            dx = path[0, ind + dind] - state[0]
            dy = path[1, ind + dind] - state[1]

            # position references
            xref[0, i] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])
            xref[1, i] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])
            xref[2, i] = target_v  # sp[ind + dind]
            xref[3, i] = normalize_angle(path[2, ind + dind] - state[3])

            # steering angle 
            dref[0, i] = 0.0
        else:
            dx = path[0, path_len - 1] - state[0]
            dy = path[1, path_len - 1] - state[1]
            xref[0, i] = dx * np.cos(-state[3]) - dy * np.sin(-state[3])
            xref[1, i] = dy * np.cos(-state[3]) + dx * np.sin(-state[3])
            xref[2, i] = 0.0  # stop? if not: #sp[ncourse - 1]
            xref[3, i] = normalize_angle(path[2, path_len - 1] - state[3])
            dref[0, i] = 0.0
    return xref, dref, ind


def get_linear_model_matrices(x_bar, u_bar):
    """
    Computes the LTI approximated state space model x' = Ax + Bu + C
    """

    x_bar[0]
    x_bar[1]
    v = x_bar[2]
    theta = x_bar[3]

    a = u_bar[0]
    delta = u_bar[1]

    ct = np.cos(theta)
    st = np.sin(theta)
    cd = np.cos(delta)
    td = np.tan(delta)

    A = np.zeros((P.N, P.N))
    A[0, 2] = ct
    A[0, 3] = -v * st
    A[1, 2] = st
    A[1, 3] = v * ct
    A[3, 2] = v * td / P.L
    A_lin = np.eye(P.N) + P.DT * A

    B = np.zeros((P.N, P.M))
    B[2, 0] = 1
    B[3, 1] = v / (P.L * cd**2)
    B_lin = P.DT * B

    f_xu = np.array([v * ct, v * st, a, v * td / P.L]).reshape(P.N, 1)
    C_lin = (
        P.DT
        * (
            f_xu - np.dot(A, x_bar.reshape(P.N, 1)) - np.dot(B, u_bar.reshape(P.M, 1))
        ).flatten()
    )

    # return np.round(A_lin,6), np.round(B_lin,6), np.round(C_lin,6)
    return A_lin, B_lin, C_lin