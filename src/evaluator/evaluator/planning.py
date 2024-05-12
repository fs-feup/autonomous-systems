import numpy as np
from custom_interfaces.msg import PathPointArray

def convert_path_to_np(path_point_array):
    path_list = []

    for point in path_point_array:
        path_list.append(np.array([point.x, point.y, 0.0]))

    return path_list