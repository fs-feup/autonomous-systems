import numpy as np

def data_association(map, observations):
    """
    Perform data association between the map and the observations.

    Args:
        map (list): The map containing landmarks in global coordinates [x1, y1, x2, y2, ...].
        observations (list): The list of observations in global coordinates [x1, y1, x2, y2, ...].

    Returns:
        list: for each observation: -2 if it should be totally disregarded, -1 if it is new, or the index
        of the x coordinate of the associated landmark in the map if it is an existing landmark.
    """
    associated_landmarks = np.array([-2] * (len(observations) / 2), dtype=int)

    return associated_landmarks