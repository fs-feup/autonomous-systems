import numpy as np
import math

def steer(self, error):
    """!
    @brief Steers the car.
    @param self The object pointer.
    @param error Error.
    """
    kp = 0.03
    kd = 0.0

    steer_angle_rate = kp*min(error, 10000000) + kd*(error - self.old_error)

    return float(steer_angle_rate)


def get_closest_point(pose, points_array):
    """!
    @brief Gets the closest point to the car.
    @param self The object pointer.
    @param pose Pose.
    @param points_array Array of points.
    @return Closest point.
    """
    position = np.array([pose[0], pose[1]]).reshape((1, 2))
    position = np.repeat(position, len(points_array), axis=0)

    dists_sqrd = np.sum((position - points_array)**2, axis=1)
    closest_index = np.argmin(dists_sqrd)

    return points_array[closest_index], closest_index


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
    angle = math.radians(pose[2])

    c_x = closest_point[0]
    c_y = closest_point[1]

    rotation_matrix = [[math.cos(angle), -math.sin(angle), 0],
                       [math.sin(angle), math.cos(angle), 0],
                       [0, 0, 1]]

    transl_vector = [[x],
                     [y],
                     [0]]

    position = [[c_x],
                [c_y],
                [0]]

    new_closest = np.dot(rotation_matrix, np.array(position)-np.array(transl_vector))

    return new_closest[0]