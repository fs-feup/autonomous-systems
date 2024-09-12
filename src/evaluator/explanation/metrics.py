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

def get_mean_squared_difference(output: np.ndarray, expected: np.ndarray) -> float:
    """!
    Computes the mean squared difference between an output output and the expected values.

    Args:
        output (list): Empirical Output.
        expected (list): Expected output.

    Returns:
        float: Mean squared difference.
    """

def compute_distance(cone1: np.array, cone2: np.array) -> float:
    """!
    Compute Euclidean distance between two cones.
    """

def build_adjacency_matrix(cones: np.array) -> np.array:
    """Build adjacency matrix based on distances between cones."""

def get_duplicates(output: np.array, threshold: float):
    """Receives a set of cones and identify the possible dupliates"""

def get_inter_cones_distance(perception_output: np.array) -> float:
    """!
    Computes the average distance between pairs of perceived cones using Minimum Spanning Tree Prim's algorithm.

    Args:
        perception_output (np.array): List of perceived cones, where each cone is represented as a numpy array.

    Returns:
        float: Average distance between pairs of perceived cones.
    """