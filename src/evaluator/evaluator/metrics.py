import numpy as np
import sys
from scipy.sparse.csgraph import minimum_spanning_tree


def get_average_difference(output: np.array, expected: np.array) -> float:
    """!
    Computes the average difference between an output output and the expected values.

    Args:
        output (np.array): Empirical Output.
        expected (np.array): Expected output.

    Returns:
        float: Average difference between empirical and expected outputs.
    """

    if len(output) == 0:
        return float("inf")

    if len(expected) == 0:
        raise ValueError(
            "No ground truth cones provided for computing average difference."
        )

    # Step 1 & 2: Compute the differences using broadcasting and vectorization
    differences = np.linalg.norm(
        output[:, np.newaxis, :] - expected[np.newaxis, :, :], axis=2
    )

    # Step 3: Find the minimum squared difference for each output_value and sum them up
    min_differences_sum: float = np.min(differences, axis=1).sum()

    # Step 4: Compute MSE
    me: float = min_differences_sum / len(output)

    return me


def get_false_positives(output: np.array, expected: np.array, threshold: float) -> int:
    """!
    Computes the number of false positives in the output compared to the expected values.

    Args:
        output (np.array): Empirical Output.
        expected (np.array): Expected output.
        threshold (float): Distance threshold to consider values as matching.

    Returns:
        int: Number of false positives.
    """
    if len(output) == 0:
        return 0

    if len(expected) == 0:
        raise ValueError(
            "No ground truth cones provided for computing false positives."
        )

    # Create a boolean array to mark the expected values that have been matched
    matched_expected = np.zeros(len(expected), dtype=bool)
    false_positives: int = 0

    for empirical_value in output:
        min_difference = sys.float_info.max
        min_index = -1

        for i, expected_value in enumerate(expected):
            difference = np.linalg.norm(empirical_value - expected_value)
            if difference < min_difference:
                min_difference = difference
                min_index = i

        if min_difference < threshold and not matched_expected[min_index]:
            matched_expected[min_index] = True
        else:
            false_positives += 1

    return false_positives


def get_mean_squared_difference(output: np.ndarray, expected: np.ndarray) -> float:
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

    # Step 1 & 2: Compute the squared differences using broadcasting and vectorization
    differences = (
        np.linalg.norm(output[:, np.newaxis, :] - expected[np.newaxis, :, :], axis=2)
        ** 2
    )

    # Step 3: Find the minimum squared difference for each output_value and sum them up
    min_differences_sum = np.min(differences, axis=1).sum()

    # Step 4: Compute MSE
    mse = min_differences_sum / len(output)

    return mse


def compute_distance(cone1, cone2) -> float:
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
