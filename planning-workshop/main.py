import os
import re
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Vehicle:
    x: float
    y: float
    angle: float


@dataclass
class TestCase:
    path_points: List[Tuple[float, float]]
    cones: List[Tuple[float, float]]
    vehicle: Vehicle
    def __str__(self):
        return (f"TestCase with {len(self.path_points)} path points, "
                f"{len(self.cones)} cones, "
                f"vehicle at ({self.vehicle.x}, {self.vehicle.y}) with angle {self.vehicle.angle}")


def read_test_file(file_path):
    path_points = []
    cones = []
    vehicle = None
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if parts[0] == 'P' and len(parts) >= 3:
                x, y = float(parts[1]), float(parts[2])
                path_points.append((x, y))
            elif parts[0] == 'C' and len(parts) >= 3:
                x, y = float(parts[1]), float(parts[2])
                cones.append((x, y))
            elif parts[0] == 'V' and len(parts) >= 4:
                x, y, angle = float(parts[1]), float(parts[2]), float(parts[3])
                vehicle = Vehicle(x, y, angle)
    if vehicle is None:
        print(f"Warning: No vehicle data in {file_path}, using default (0,0,0)")
        vehicle = Vehicle(0.0, 0.0, 0.0)
    return TestCase(path_points, cones, vehicle)


def process_folder(folder_path):
    test_cases = []
    if not os.path.isdir(folder_path):
        print(f"Error: Folder '{folder_path}' does not exist.")
        return test_cases
    files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]
    for file_name in files:
        input_path = os.path.join(folder_path, file_name)
        try:
            test_case = read_test_file(input_path)
            test_cases.append(test_case)
            print(f"Processed '{file_name}': {test_case}")
        except Exception as e:
            print(f"Error processing '{file_name}': {e}")
    return test_cases


#####################################################################
###################### IMPLEMENT THIS FUNCTION ######################
#####################################################################
def border_estimation(test_case):
    """
    IMPLEMENT YOUR BORDER ESTIMATION ALGORITHM HERE
    
    This function should separate the cones into left and right borders.
    
    Input:
        test_case: A TestCase object containing path points, cones, and vehicle state
    
    Output:
        left_cones: List of (x,y) tuples representing the left border cones
        right_cones: List of (x,y) tuples representing the right border cones
    
    Notes:
        - Classify each cone as either left or right border
        - The visualization code will automatically display:
            - Left cones in blue
            - Right cones in yellow
        - No need to modify any other part of the code
    """
    # This is a placeholder implementation
    # Replace this with your actual border estimation algorithm
    midpoint = len(test_case.cones) // 2
    left_cones = test_case.cones[:midpoint]
    right_cones = test_case.cones[midpoint:]
    
    return left_cones, right_cones



def visualize_track(test_case, left_cones, right_cones):
    plt.figure(figsize=(12, 8))
    if test_case.path_points:
        path_x, path_y = zip(*test_case.path_points)
        plt.plot(path_x, path_y, 'g-', label='Path')
    if left_cones:
        left_x, left_y = zip(*left_cones)
        plt.scatter(left_x, left_y, color='blue', s=100, marker='^', label='Left cones')
    if right_cones:
        right_x, right_y = zip(*right_cones)
        plt.scatter(right_x, right_y, color='yellow', s=100, marker='^', edgecolors='black', label='Right cones')
    vehicle = test_case.vehicle
    plt.scatter(vehicle.x, vehicle.y, color='red', s=150, marker='o', label='Vehicle')
    arrow_length = 2.0
    dx = arrow_length * np.cos(vehicle.angle)
    dy = arrow_length * np.sin(vehicle.angle)
    plt.arrow(vehicle.x, vehicle.y, dx, dy, head_width=0.5, head_length=0.7, fc='red', ec='red')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Track Visualization')
    plt.legend()
    return plt


def main(folder_path):
    test_cases = process_folder(folder_path)
    for i, test_case in enumerate(test_cases):
        left_cones, right_cones = border_estimation(test_case)
        plot = visualize_track(test_case, left_cones, right_cones)
        plot.savefig(f"./planning-workshop/output/track_visualization_{i+1}.png")
        print(f"Visualization saved as track_visualization_{i+1}.png")
        plot.close()


if __name__ == "__main__":
    folder_path = "./planning-workshop/input"
    main(folder_path)