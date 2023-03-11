import numpy as np
import math

def get_closest_point(position, points_array):
    """!
    @brief Gets the closest point to the given position.
    @param self The object pointer.
    @param position List with x and y coordinates.
    @param points_array Numpy array of points.
    @return Closest point to position in points_array.
    """

    dists_sqrd = np.sum((points_array - position)**2, axis=1)
    return points_array[np.argmin(dists_sqrd)]

def right_or_left(pose, closest_point):
    """!
    @brief Checks if the car is to the right or left of the path.
    @param self The object pointer.
    @param pose Pose.
    @param closest_point Closest point.
    @return Error.
    """
    x = pose[0]
    y = pose[1]
    angle = pose[2]

    c_x = closest_point[0]
    c_y = closest_point[1]

    rotation_matrix = [[math.cos(angle), math.sin(angle), 0],
                       [-math.sin(angle), math.cos(angle), 0],
                       [0, 0, 1]]

    transl_vector = [[x],
                     [y],
                     [0]]

    position = [[c_x],
                [c_y],
                [0]]

    new_closest = np.dot(rotation_matrix, np.array(position)-np.array(transl_vector))

    return new_closest[1]