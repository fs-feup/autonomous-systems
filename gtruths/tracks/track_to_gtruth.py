import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import csv
import argparse


def read_csv(filename):
    initial_points = []
    with open(filename, "r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header if present
        for row in reader:
            x, y, _ = row[:3]  # Assuming tag is in the third column
            initial_points.append((float(x), float(y)))
    return initial_points


class PointCollector:
    def __init__(self, initial_points):
        self.fig, self.ax = plt.subplots()
        self.points = list(initial_points)
        self.added_points = []  # Track the added points separately
        self.ax.set_aspect("equal")
        self.ax.set_title("Click to add points, drag to pan, scroll to zoom, press 'u' to undo")
        self.ax.scatter(*zip(*self.points), color="blue")
        self.is_dragging = False
        self.clicked = False
        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self.onclick)
        self.cid_scroll = self.fig.canvas.mpl_connect("scroll_event", self.onscroll)
        self.cid_release = self.fig.canvas.mpl_connect(
            "button_release_event", self.onrelease
        )
        self.cid_key = self.fig.canvas.mpl_connect("key_press_event", self.onkey)
        plt.show()

    def onclick(self, event):
        if event.inaxes == self.ax:
            self.clicked = True

    def onscroll(self, event):
        if event.inaxes == self.ax:
            scale = 1.1 if event.button == "down" else 1 / 1.1
            self.ax.set_xlim(
                event.xdata - (event.xdata - self.ax.get_xlim()[0]) * scale,
                event.xdata + (self.ax.get_xlim()[1] - event.xdata) * scale,
            )
            self.ax.set_ylim(
                event.ydata - (event.ydata - self.ax.get_ylim()[0]) * scale,
                event.ydata + (self.ax.get_ylim()[1] - event.ydata) * scale,
            )
            self.fig.canvas.draw()
            self.clicked = False

    def onrelease(self, event):
        if event.inaxes == self.ax:
            if self.clicked and not self.is_dragging:
                self.added_points.append(
                    (event.xdata, event.ydata)
                )  # Track added points
                self.ax.scatter(event.xdata, event.ydata, color="red")
                self.fig.canvas.draw()
            self.clicked = False
            self.is_dragging = False

    def onkey(self, event):
        if event.key == 'u' and self.added_points:
            self.undo_last_point()

    def undo_last_point(self):
        if self.added_points:
            self.added_points.pop()
            self.ax.cla()
            self.ax.set_aspect("equal")
            self.ax.set_title("Click to add points, scroll to zoom")
            self.ax.scatter(*zip(*self.points), color="blue")
            if self.added_points:
                self.ax.scatter(*zip(*self.added_points), color="red")
            self.fig.canvas.draw()

    def calculate_angle(self, points):
        """Calculate the average angle given a list of points."""
        if len(points) < 3:
            raise ValueError("At least three points are required to calculate an angle.")
        
        angles = []
        for i in range(1, len(points) - 1):
            BA = np.array(points[i-1]) - np.array(points[i])
            BC = np.array(points[i+1]) - np.array(points[i])
            cosine_angle = np.dot(BA, BC) / (np.linalg.norm(BA) * np.linalg.norm(BC))
            cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
            angle = np.arccos(cosine_angle)
            angles.append(np.degrees(angle))
        
        return np.mean(angles)

    def determine_velocity(self, angle, max_velocity=5, min_velocity=1):
        """Determine the velocity based on the angle. Sharper angles result in lower velocities."""
        if angle > 180:
            return max_velocity
        elif angle < 135:
            return min_velocity
        else:
            # Linear interpolation between min_velocity and max_velocity (could tuner further)
            return min_velocity + (max_velocity - min_velocity) * (angle - 135) / (180 - 135)
        
    def export_to_csv(self, filename):
        os.makedirs(os.path.dirname(filename), exist_ok=True)  # Ensure the directory exists
        
        # Calculate velocities based on angles
        velocities = [0] * len(self.added_points)
        window_size = 5
        for i in range(1, len(self.added_points) - 1):
            start_index = max(0, i - window_size // 2)
            end_index = min(len(self.added_points), i + window_size // 2 + 1)
            angle = self.calculate_angle(self.added_points[start_index:end_index])
            velocities[i] = self.determine_velocity(angle)
            

        velocities[0] = velocities[-1] = 5
        
        df = pd.DataFrame(self.added_points, columns=["x", "y"])
        df["z"] = velocities
        df.to_csv(filename, index=False)
        


def main():
    parser = argparse.ArgumentParser(
        description="Create ground truth points for a track."
    )
    parser.add_argument(
        "--sim", type=str, default="fsds", help="Simulator type (fsds, eufs, pacsim)"
    )
    parser.add_argument(
        "--track_name",
        type=str,
        default="track1.csv",
        help="Name of the track CSV file",
    )

    args = parser.parse_args()
    sim = args.sim
    track_name = args.track_name

    # Get the current directory
    current_directory = os.getcwd()

    # Concatenate current directory with the provided filename
    filename = os.path.join(
        current_directory, "gtruths", "tracks", sim, track_name[:-4], track_name
    )

    initial_points = read_csv(filename)

    pc = PointCollector(initial_points)

    filename = input("Please enter the name of the gtruth to save the points (e.g., skidpad): ")
    final_file_name = os.path.join(
        "gtruths", "tracks", sim, filename, f"{filename}_gtruth.csv"
    )

    pc.export_to_csv(final_file_name)


if __name__ == "__main__":
    main()
