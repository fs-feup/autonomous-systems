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
    return points_array[np.argmin(dists_sqrd)]

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