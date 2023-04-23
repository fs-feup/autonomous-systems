import numpy as np
import math

from scipy import interpolate

def get_closest_point(position, points_array):
    """!
    @brief Gets the closest point to the given position.
    @param position List with x and y coordinates.
    @param points_array Numpy array of points.
    @return Closest point to position in points_array.
    """

    dists_sqrd = np.sum((points_array - position)**2, axis=1)
    closest_index = np.argmin(dists_sqrd)

    return points_array[closest_index], closest_index


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

    filtered_points = points_array[max(closest_index-win_amplitude, 0):min(closest_index+win_amplitude, 26), :]

    tck, u = interpolate.splprep([filtered_points[:, 0], filtered_points[:, 1]], s=0, per=False)
    x0, y0 = interpolate.splev(np.linspace(0, 1, new_closest_index), tck)

    new_points_array = np.concatenate((x0[:, np.newaxis], y0[:, np.newaxis]), axis=1)

    _, new_closest_index = get_closest_point([x_car, y_car], new_points_array)

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

    yaw_track = math.acos(np.dot(track_vector, i_vector)/math.sqrt(track_vector[0]**2 + track_vector[1]**2))

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
    

def get_reference_speed(speeds, closest_index):
    return speeds[closest_index]

def get_speed_error(lin_speed, ref_speed):
    return ref_speed - lin_speed

