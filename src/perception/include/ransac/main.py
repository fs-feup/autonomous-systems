
from math import acos
from random import random, choice
from time import time

class Point:
    """
    This class represents a point in 3D space.
    """

    def __init__(self, x: float, y: float, z: float):
        """
        Constructor for the Point class.

        :param x: X coordinate of the point.
        :param y: Y coordinate of the point.
        :param z: Z coordinate of the point.
        """
        self.x = x
        self.y = y
        self.z = z
    def distance_to_plane(self, plane):
        """
        Calculate the distance from the point to a plane.

        :param plane: A Plane object representing the plane.
        :return: Distance from the point to the plane.
        """
        # Plane equation: Ax + By + Cz + D = 0
        A, B, C = plane.normal
        D = -(A * plane.point[0] + B * plane.point[1] + C * plane.point[2])
        return abs(A * self.x + B * self.y + C * self.z + D) / (A**2 + B**2 + C**2)**0.5

class PointCloud:
    """
    This class represents a point cloud, which is a collection of points in 3D space.
    """

    def __init__(self, points: list[Point]):
        """
        Constructor for the PointCloud class.

        :param points: A list of tuples representing the points in the point cloud.
        """
        self.points = points

class Plane:
    """
    This class represents a plane in 3D space defined by its normal vector and a point on the plane.
    """

    def __init__(self, normal: tuple, point: tuple):
        """
        Constructor for the Plane class.

        :param normal: A tuple representing the normal vector of the plane.
        :param point: A tuple representing a point on the plane.
        """
        self.normal = normal
        self.point = point
    


class Ransac:
    """
    This class implements the RANSAC algorithm for ground removal in point clouds.
    It inherits from the GroundRemoval interface.
    """

    def __init__(self, epsilon: float, n_tries: int):
        """
        Constructor for the RANSAC ground removal algorithm.

        :param epsilon: Epsilon threshold for ground removal.
        :param n_tries: Number of RANSAC iterations.
        """
        self.epsilon = epsilon
        self.n_tries = n_tries

    def remove_ground(self, point_cloud: PointCloud):
        """
        Perform ground removal using the RANSAC algorithm.

        :param point_cloud: Input point cloud data.
        :return: Point cloud with ground points removed.
        """
        # Implementation of the RANSAC algorithm goes here
        pass
    
    def calculate_plane(self, point_cloud: PointCloud, target_plane: Plane = None, angle_diff: float = 0):
        """
        Calculate the best-fit plane for the given point cloud using RANSAC.
        :param point_cloud: Input point cloud data.
        :param base_plane: Initial guess for the plane.
        :return: Best-fit plane parameters.
        """
        # Implementation of the RANSAC plane fitting goes here
        self.point_cloud = point_cloud
        self.target_plane = target_plane
        self.angle_diff = angle_diff
        
        if len(point_cloud.points) < 3:
            raise ValueError("Point cloud must contain at least 3 points to fit a plane.")

        best_plane = None
        best_plane_inliers = 0
        best_plane_max_deviation = 0
        for i in range(self.n_tries):
            while True:
                # Randomly sample points from the point cloud
                # Fit a plane to the sampled points
                # Calculate the number of inliers based on the epsilon threshold
                # If the number of inliers is greater than the best found so far, update the best plane
                while True:
                    sampled_points = [choice(point_cloud.points) for _ in range(3)]
                    (p1, p2, p3) = sampled_points
                    if p1 != p2 and p1 != p3 and p2 != p3:
                        break
                # Fit a plane to the sampled points
                # Calculate the normal vector of the plane
                # Use the cross product of two vectors formed by the points
                v1 = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
                v2 = (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)
                normal = (v1[1] * v2[2] - v1[2] * v2[1],
                            v1[2] * v2[0] - v1[0] * v2[2],
                            v1[0] * v2[1] - v1[1] * v2[0])
                magnitude = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
                normal = (normal[0] / magnitude, normal[1] / magnitude, normal[2] / magnitude)
                res = Plane(normal, (p1.x, p1.y, p1.z))
                # Calculate the angle difference between the plane normal and the target plane normal
                if target_plane:
                    dot_product = (normal[0] * target_plane.normal[0] +
                                    normal[1] * target_plane.normal[1] +
                                    normal[2] * target_plane.normal[2])
                    angle = abs(dot_product) / ((normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5 *
                                                (target_plane.normal[0]**2 + target_plane.normal[1]**2 + target_plane.normal[2]**2)**0.5) # Avoid division by zero
                    angle = abs(angle)
                    if angle > 1.0:  # Handle floating-point precision issues
                        angle = 1.0
                    angle_diff_radians = acos(angle)
                    angle_diff_degrees = angle_diff_radians * (180 / 3.141592653589793)
                    if angle_diff_degrees <= angle_diff:
                        break
                else:
                    break
            # Calculate the number of inliers based on the epsilon threshold
            inliers = 0
            max_deviation = 0
            for point in point_cloud.points:
                d = point.distance_to_plane(res)
                if d < self.epsilon:
                    inliers += 1
                    if d > max_deviation:
                        max_deviation = d
            if best_plane is None or inliers > best_plane_inliers:
                best_plane = res
                best_plane_inliers = inliers
                best_plane_max_deviation = max_deviation
                print(res.normal, res.point)
                print(f"New best plane found with {inliers} inliers and max deviation {max_deviation}")
            elif inliers == best_plane_inliers and max_deviation < best_plane_max_deviation:
                best_plane = res
                best_plane_inliers = inliers
                best_plane_max_deviation = max_deviation
                print(f"New best plane found with max deviation {max_deviation}")
            # If the number of inliers is greater than the best found so far, update the best plane
        self.best_plane = best_plane
    
    def ground_removal(self):
        """
        Remove ground points from the point cloud based on the best-fit plane.

        :param point_cloud: Input point cloud data.
        :param target_plane: Target plane for ground removal.
        :param angle_diff: Angle difference threshold for plane matching.
        :return: Point cloud with ground points removed.
        """
        for point in self.point_cloud.points:
            distance = point.distance_to_plane(self.best_plane)
            if distance < self.epsilon:
                self.point_cloud.points.remove(point)
        
def main():
    """
    Main function to demonstrate the usage of the Ransac class.
    """
    # Example point cloud data - points on x,y plane with slight z deviation
    points = [Point(random() * 20 - 10, random() * 20 - 10, random() * 0.4 - 0.2) for _ in range(100)] + [Point(random() * 50 - 10, random() * 50 - 10, random() * 50 - 10) for _ in range(100)] + [Point(random() * 20 - 10, random() * 20 - 10, random() * 0.4 - 0.2) for _ in range(100)]
    point_cloud = PointCloud(points)
    print(f"Generated point cloud with {len(point_cloud.points)} points.")
    start_time = time()
    target = Plane((0, 0, 1), (0, 0, 0))  # Target plane normal and point
    target_diff = 20  # Angle difference threshold
    # Create RANSAC instance and perform ground removal
    ransac = Ransac(epsilon=0.2, n_tries=100)
    ransac.calculate_plane(point_cloud, target, target_diff)
    ransac.ground_removal()
    end_time = time()
    print(f"Ground removal completed in {(end_time - start_time)*1000:.2f} miliseconds")
    print(f"Final point cloud has {len(point_cloud.points)} points after ground removal.")
    
    

if __name__ == "__main__":
    main()