import matplotlib.pyplot as plt
import pandas as pd
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
        self.ax.set_title("Click to add points, drag to pan, scroll to zoom")
        self.ax.scatter(*zip(*self.points), color="blue")
        self.is_dragging = False
        self.clicked = False
        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self.onclick)
        self.cid_scroll = self.fig.canvas.mpl_connect("scroll_event", self.onscroll)
        self.cid_drag = self.fig.canvas.mpl_connect("motion_notify_event", self.onmove)
        self.cid_release = self.fig.canvas.mpl_connect(
            "button_release_event", self.onrelease
        )
        plt.show()

    def onclick(self, event):
        if event.inaxes == self.ax:
            self.clicked = True

    def onscroll(self, event):
        if event.inaxes == self.ax:
            scale = 1.1 if event.button == "up" else 1 / 1.1
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

    def onmove(self, event):
        if event.inaxes == self.ax and event.button == 1:  # left button
            self.is_dragging = True
            self.ax.set_xlim(
                self.ax.get_xlim() - (event.x - event.xdata),
                self.ax.get_xlim() - (event.x - event.xdata),
            )
            self.ax.set_ylim(
                self.ax.get_ylim() - (event.y - event.ydata),
                self.ax.get_ylim() - (event.y - event.ydata),
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

    def export_to_csv(self, filename):
        df = pd.DataFrame(
            self.added_points, columns=["x", "y"]
        )  # Export only the added points
        df["z"] = 0
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
    final_file_name = os.path.join(
        "gtruths", "tracks", sim, track_name[:-4], f"{track_name[:-4]}_gtruth.csv"
    )

    input(
        f"The following step will overwrite the file {final_file_name} \nTo stop this, press Ctrl + C followed by Enter; To continue, press Enter \n"
    )
    pc.export_to_csv(final_file_name)


if __name__ == "__main__":
    main()
