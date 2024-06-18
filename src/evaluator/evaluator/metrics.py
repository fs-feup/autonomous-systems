import numpy as np
import sys
from scipy.sparse.csgraph import minimum_spanning_tree


def get_average_difference(output: np.array, expected: np.array):
    """!
    Computes the average difference between an output output and the expected values.

    Args:
        output (np.array): Empirical Output.
        expected (np.array): Expected output.

    Returns:
        float: Average difference between empirical and expected outputs.
    """
    sum_error: float = 0
    count: int = 0

    if len(output) == 0:
        return float("inf")

    if len(expected) == 0:
        raise ValueError(
            "No ground truth cones provided for computing average difference."
        )

    for empirical_value in output:
        min_difference: float = sys.float_info.max

        for expected_value in expected:

            difference: float = np.linalg.norm(empirical_value - expected_value)
            if difference < min_difference:
                min_difference = difference

        sum_error += min_difference
        count += 1

    average = sum_error / count
    return average


def get_mean_squared_difference(output: list, expected: list):
    """!
    Computes the mean squared difference between an output output and the expected values.

    Args:
        output (list): Empirical Output.
        expected (list): Expected output.

    Returns:
        float: Mean squared difference.
    """
    if output is None:
        raise ValueError("No perception output provided.")
    if expected is None:
        raise ValueError("No ground truth cones provided.")

    mse_sum = 0
    for output_value in output:
        min_difference_sq = sys.float_info.max

        for expected_value in expected:
            difference_sq = np.linalg.norm(output_value - expected_value) ** 2
            if difference_sq < min_difference_sq:
                min_difference_sq = difference_sq

        mse_sum += min_difference_sq

    mse = mse_sum / len(output)
    return mse


def compute_distance(cone1, cone2):
    """!
    Compute Euclidean distance between two cones.
    """
    return np.linalg.norm(cone1 - cone2)


def build_adjacency_matrix(cones):
    """!
    Build adjacency matrix based on distances between cones.
    """
    size = len(cones)
    adjacency_matrix = np.zeros((size, size))
    for i in range(size):
        for j in range(i + 1, size):
            distance_ij = compute_distance(cones[i], cones[j])
            adjacency_matrix[i][j] = distance_ij
            adjacency_matrix[j][i] = distance_ij
    return adjacency_matrix


def get_inter_cones_distance(perception_output: list):
    """!
    Computes the average distance between pairs of perceived cones using Minimum Spanning Tree Prim's algorithm.

    Args:
        perception_output (list): List of perceived cones, where each cone is represented as a numpy array.

    Returns:
        float: Average distance between pairs of perceived cones.
    """

    adjacency_matrix: list = build_adjacency_matrix(perception_output)

    mst = minimum_spanning_tree(adjacency_matrix)
    mst_sum = mst.sum()

    num_pairs = np.count_nonzero(mst.toarray().astype(float))

    if num_pairs == 0:
        return 0
    else:
        average_distance = mst_sum / num_pairs
        return average_distance
