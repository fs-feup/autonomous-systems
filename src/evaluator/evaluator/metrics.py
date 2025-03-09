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

    differences = np.linalg.norm(
        output[:, np.newaxis, :] - expected[np.newaxis, :, :], axis=-1
    )

    matched_output = np.full(len(output), False)  # Track which outputs are matched
    matched_expected = np.full(len(expected), False)  # Track expected matches

    for i in range(len(output)):
        for j in range(len(expected)):
            if not matched_expected[j] and differences[i, j] < threshold:
                matched_output[i] = True
                matched_expected[j] = True
                break

    true_positives = np.sum(matched_output)

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


def compute_closest_distances(arr1: np.ndarray, arr2: np.ndarray) -> np.ndarray:
    """!
    Computes the distance between each element in arr2 and the closest element in arr1.

    Args:
        arr1 (np.ndarray): First array of positions.
        arr2 (np.ndarray): Second array of positions.

    Returns:
        np.ndarray: Array of distances between each element in arr2 and the closest element in arr1.
    """
    # Extract the x and y positions
    arr1_xy = arr1[:, :2]
    arr2_xy = arr2[:, :2]

    # Calculate the squared Euclidean distances
    distances = np.linalg.norm(
        arr2_xy[:, np.newaxis, :] - arr1_xy[np.newaxis, :, :], axis=2
    )

    # Find the minimum distances for each element in arr2
    closest_distances = np.min(distances, axis=1)

    return closest_distances


def get_average_error(values: np.array) -> float:
    """!
    Computes the average of a list of values.

    Args:
        values (np.array): List of values.

    Returns:
        float: Average of the values.
    """
    if len(values) == 0:
        return 0.0

    return np.mean(values)


def get_mean_squared_error(values: np.array) -> float:
    """!
    Computes the mean squared value of a list of values.

    Args:
        values (np.array): List of values.

    Returns:
        float: Mean squared value of the values.
    """
    if len(values) == 0:
        return 0.0

    return np.mean(values**2)


def get_root_mean_squared_error(values: np.array) -> float:
    """!
    Computes the root mean squared error of a list of values.

    Args:
        values (np.array): List of values.

    Returns:
        float: Root mean squared error of the values.
    """
    if len(values) == 0:
        return 0.0

    return np.sqrt(np.mean(values**2))
