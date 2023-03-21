import numpy as np
import math

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
    except IndexError as e:
        closest1 = points_array[closest_index - 1]
        closest2 = points_array[closest_index]

    yaw_track = math.atan((closest1[1] - closest2[1])/(closest1[0] - closest2[0]))

    return yaw_track - yaw_car


def get_cte(closest_index, points_array, pose_car):
    x_car = pose_car[0]
    y_car = pose_car[1]
    yaw_car = pose_car[2]

    closest1 = points_array[closest_index]
    x_track = closest1[0]
    y_track = closest1[1]

    try:
        closest2 = points_array[closest_index + 1]
    except IndexError as e:
        closest1 = points_array[closest_index - 1]
        closest2 = points_array[closest_index]

    yaw_track = math.atan(((closest1[1] - closest2[1]) + 0.00)/(closest1[0] - closest2[0]))

    alpha = yaw_car - yaw_track
    alpha_car = math.tan(yaw_car)
    alpha_track = math.tan(yaw_track)

    # get intersection coordinates
    x_int = ((y_car - y_track) - (alpha_car*x_car - alpha_track*x_track)) / (alpha_track - alpha_car)
    y_int = alpha_track*x_int + y_track - alpha_track*x_track

    car_to_int = math.sqrt((x_car - x_int)**2 + (y_car - y_int)**2)

    return math.sin(alpha)*car_to_int


def get_cte2(closest_index, points_array, pose_car):
    x_car = pose_car[0]
    y_car = pose_car[1]
    yaw_car = pose_car[2]

    closest1 = points_array[closest_index]
    x_track = closest1[0]
    y_track = closest1[1]

    try:
        closest2 = points_array[closest_index + 1]
    except IndexError as e:
        closest1 = points_array[closest_index - 1]
        closest2 = points_array[closest_index]

    yaw_track = math.atan(((closest1[1] - closest2[1]) + 0.00)/(closest1[0] - closest2[0]))

    lambda2_num = (x_track - x_car)/math.cos(yaw_car) - (y_track - y_car)/math.sin(yaw_car)
    lambda2_den = math.sin(yaw_track)/math.sin(yaw_car) - math.cos(yaw_track)/math.cos(yaw_car)

    lambda2 = lambda2_num / lambda2_den

    x_int = math.cos(yaw_track)*lambda2 + x_track
    y_int = math.sin(yaw_track)*lambda2 + y_track

    car_to_int = math.sqrt((x_car - x_int)**2 + (y_car - y_int)**2)

    return math.sin(yaw_car - yaw_track)*car_to_int
