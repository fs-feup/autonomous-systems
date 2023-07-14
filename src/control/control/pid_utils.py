import numpy as np
import math
from scipy import interpolate
from .config import Params

P = Params()

def get_closest_point(state, path, old_nn_idx=None, search_window=None):
    """
    Computes the index of the waypoint closest to vehicle
    """

    if old_nn_idx and search_window:
        windowed_path = path[max(old_nn_idx - search_window, 0): 
                            min(old_nn_idx + search_window, path.shape[0]), :]

        index_offset = max(old_nn_idx - search_window, 0)
    else:

        windowed_path = path
        index_offset = 0
    
    dx = state[0] - windowed_path[:, 0]
    dy = state[1] - windowed_path[:, 1]

    dist = np.hypot(dx, dy)
    
    nn_idx = np.argmin(dist) + index_offset
    
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


def get_position_error(pose, closest_point):
    """!
    @brief Calculates the position error between pose and closest_point.
    @param pose Pose information.
    @param closest_point Coordinates off the closest point.
    @return Position error.
    """

    angle = pose[2]

    rotation_matrix = [[math.cos(angle), math.sin(angle), 0],
                       [-math.sin(angle), math.cos(angle), 0],
                       [0, 0, 1]]

    transl_vector = np.array([
        [pose[0]],
        [pose[1]],
        [0]
    ])

    position = np.array([
        [closest_point[0]],
        [closest_point[1]],
        [0]
    ])

    new_closest = np.dot(rotation_matrix, position-transl_vector)

    return new_closest[1][0]


def get_orientation_error(closest_index, points_array, yaw_car):
    closest1 = points_array[closest_index]

    try:
        closest2 = points_array[closest_index + 1]
    except IndexError:
        closest1 = points_array[closest_index - 1]
        closest2 = points_array[closest_index]

    yaw_track = math.atan((closest1[1] - closest2[1])/(closest1[0] - closest2[0]))

    return yaw_track - yaw_car


def get_cte(closest_index, points_array, pose_car):
    """!
    @brief Calculates the cross track error (CTE). 
    CTE is defined as the distance between the car and the line tangent to the 
    closest point of the track to the car.
    @param closest_index position of closest point of the track to the car.
    @param points_array List containing PointArray. Represent track points.
    @param pose_array List containing car x position, y position and yaw.
    @return CTE.
    """
    x_car = pose_car[0]
    y_car = pose_car[1]
    yaw_car = pose_car[2]

    spline_window = 10
    win_amplitude = int(spline_window/2)
    n_new_points = 50

    filtered_points = points_array[max(closest_index-win_amplitude, 0):
        min(closest_index+win_amplitude, len(points_array)), :]

    tck, u = interpolate.splprep([filtered_points[:, 0], filtered_points[:, 1]],
                                  s=0, per=False)
    x0, y0 = interpolate.splev(np.linspace(0, 1, n_new_points), tck)

    new_points_array = np.concatenate((x0[:, np.newaxis], y0[:, np.newaxis]), axis=1)

    new_closest_index = get_closest_point([x_car, y_car], new_points_array)

    closest1 = new_points_array[new_closest_index]
    x_track = closest1[0]
    y_track = closest1[1]
     
    try:
        closest2 = new_points_array[new_closest_index + 1]
    except IndexError:
        closest1 = new_points_array[new_closest_index - 1]
        closest2 = new_points_array[new_closest_index]
    
    track_vector = np.array([closest2[0] - closest1[0], closest2[1] - closest1[1]])
    i_vector = np.array([1, 0])

    np.linalg.norm(track_vector)

    yaw_track = math.acos(np.dot(track_vector, i_vector) /
                          math.sqrt(track_vector[0]**2 + track_vector[1]**2))

    if track_vector[1] < 0:
        yaw_track = 2*math.pi - yaw_track

    D = np.array(
        [[math.cos(yaw_car), -math.cos(yaw_track)],
         [math.sin(yaw_car), -math.sin(yaw_track)]]
    )

    Dx = np.array(
        [[(x_track - x_car), -math.cos(yaw_track)],
         [(y_track - y_car), -math.sin(yaw_track)]]
    )

    Dy = np.array(
        [[math.cos(yaw_car), (x_track - x_car)],
         [math.sin(yaw_car), (y_track - y_car)]]
    )

    lambda2 = np.linalg.det(Dy)/np.linalg.det(D)
    lambda1 = np.linalg.det(Dx)/np.linalg.det(D)

    x_intersect  = math.cos(yaw_track)*lambda2 + x_track
    y_intersect = math.sin(yaw_track)*lambda2 + y_track

    car_to_intersect = math.sqrt((x_car - x_intersect)**2 + (y_car - y_intersect)**2)

    mult = 1 if lambda1 > 0 else -1

    return math.sin(yaw_car - yaw_track)*car_to_intersect*mult
    

def get_speed_command(speeds, closest_index):
    return speeds[closest_index]


def steer(pos_error, yaw_error, ct_error, old_error):
    """!
    @brief Steers the car.
    @param self The object pointer.
    @param pos_error Position Error.
    @param yaw_error Orientation Error.
    @param ct_error Cross Track Error.
    """

    # compute global error
    error = 0*pos_error + 0*yaw_error + 1*ct_error

    # calculate steering angle command
    steer_angle = P.kp_steer*error + P.kd_steer*(error - old_error)
    
    # save old error for derivative of error calculation
    old_error = error

    # save reference in node's attribute to be accessed by other methods
    return np.clip(float(steer_angle), -P.MAX_STEER, P.MAX_STEER), old_error


def get_torque_break_commands(actual_speed, desired_speed, old_error):
    error = desired_speed - actual_speed

    torque_req = 0
    break_req = 0

    # calculate steering angle command
    if error > 0:
        # torque
        torque_req = max(P.kp_torque*error + P.kd_torque*(error - old_error), 0)

    else:
        # break
        break_req = -min(P.kp_break*error + P.kd_break*(error - old_error), 0)

    return np.clip(torque_req, 0, P.MAX_TORQUE), np.clip(break_req, 0, P.MAX_BREAK), error
