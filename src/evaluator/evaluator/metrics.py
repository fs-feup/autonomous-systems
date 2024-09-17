import numpy as np
import sys
from scipy.sparse.csgraph import minimum_spanning_tree
import rclpy
import rclpy.logging


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


def get_false_positives(
    output: np.ndarray, expected: np.ndarray, threshold: float
) -> int:
    """!
    Computes the number of false positives in the output compared to the expected values.

    Args:
        output (np.ndarray): Empirical Output.
        expected (np.ndarray): Expected output.
        threshold (float): Distance threshold to consider values as matching.

    Returns:
        int: Number of false positives.
    """
    if len(output) == 0:
        return 0

    if len(expected) == 0:
        raise ValueError(
            "No ground truth values provided for computing false positives."
        )

    true_positives = 0

    differences = np.linalg.norm(
        output[:, np.newaxis, :] - expected[np.newaxis, :, :], axis=-1
    )

    # TODO: this function does not work well
    matched_expected = np.any(differences < threshold, axis=1)

    true_positives = np.sum(matched_expected)

    return max(0, len(output) - true_positives)


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


def compute_distance(cone1: np.array, cone2: np.array) -> float:
    """!
    Compute the Euclidean distance between two cones.
    Args:
        cone1 (np.array): The coordinates of the first cone.
        cone2 (np.array): The coordinates of the second cone.
    Returns:
        float: The Euclidean distance between the two cones.
    """

    return np.linalg.norm(cone1 - cone2)


def build_adjacency_matrix(cones: np.array) -> np.array:
    """
    Build an adjacency matrix based on the distances between cones.

    Args:
        cones (np.array): An array containing the coordinates of cones.

    Returns:
        np.array: The adjacency matrix representing the distances between cones.
    """
    """Build adjacency matrix based on distances between cones."""

    num_cones = cones.shape[0]

    if num_cones == 0:
        return np.array([])

    differences = cones[:, np.newaxis] - cones[np.newaxis, :]
    distances = np.linalg.norm(differences, axis=-1)

    np.fill_diagonal(distances, 0.0)

    return distances


def get_duplicates(output: np.array, threshold: float) -> int:
    """
    Receives a set of cones and identifies the possible duplicates.

    Args:
        output (np.array): The set of cones.
        threshold (float): The threshold value to consider cones different or duplicates.

    Returns:
        int: The number of possible duplicates.

    """

    adjacency_matrix = build_adjacency_matrix(output)
    num_duplicates = np.sum(np.tril(adjacency_matrix < threshold, k=-1))

    return num_duplicates


def get_inter_cones_distance(perception_output: np.array) -> float:
    """!
    Computes the average distance between pairs of perceived cones using Minimum Spanning Tree Prim's algorithm.

    Args:
        perception_output (np.array): List of perceived cones, where each cone is represented as a numpy array.

    Returns:
        float: Average distance between pairs of perceived cones.
    """

    adjacency_matrix: np.array = build_adjacency_matrix(perception_output)

    adjacency_matrix: list[np.array] = [
        adjacency_matrix[i] for i in range(len(adjacency_matrix))
    ]

    if len(adjacency_matrix) == 0:
        return 0

    mst = minimum_spanning_tree(adjacency_matrix)
    mst_sum = mst.sum()

    num_pairs = np.count_nonzero(mst.toarray().astype(float))

    if num_pairs == 0:
        return 0.0
    else:
        average_distance = mst_sum / num_pairs
        return float(average_distance)
