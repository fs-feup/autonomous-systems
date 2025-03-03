import matplotlib.pyplot as plt
import os
import math
import numpy as np

class PointTracker:
    """
    A class to interactively track and mark points on a matplotlib plot.

    The tool supports:
      - Marking yellow and blue cones (track positions).
      - Setting an initial position.
      - Defining a target for orientation.
      - Selecting two points for a final rectangular box.
    
    Data is saved to a file when the user finishes.
    """
    
    def __init__(self):
        # Initialize figure and axes.
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        
        # Cone point lists and current color.
        self.x_points = []
        self.y_points = []
        self.colors = []
        self.current_color = None
        self.scatter = None

        # Initial position and orientation variables.
        self.init_pos = None
        self.temp_target = None
        self.init_scatter = None
        self.target_scatter = None
        self.orientation_line = None
        self.theta = None

        # Final box variables.
        self.final_box_points = []
        self.final_box_plot = None

        # Disable default key handlers.
        self.fig.canvas.mpl_disconnect(self.fig.canvas.manager.key_press_handler_id)
        
        # Configure plot and connect events.
        self.configure_plot()
        self.setup_events()
        self.mode = 'none'
        self.fig.canvas.draw()

    def configure_plot(self):
        """Configure the matplotlib plot with title, labels, gridlines, and limits."""
        title = ('Track Setup Tool\n'
                 'Y: Yellow Cones, B: Blue Cones\n'
                 'P: Initial Position, T: Target for Orientation\n'
                 'F: Final Box Points\n'
                 'Right-click to Finish')
        self.ax.set_title(title)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.grid(True)
        self.ax.minorticks_on()
        self.ax.grid(which='minor', linestyle=':', linewidth=0.5)
        self.ax.grid(which='major', linestyle='-', linewidth=1)
        self.ax.set_xlim(-5, 40)
        self.ax.set_ylim(-5, 40)
        self.ax.set_autoscale_on(False)

    def setup_events(self):
        """Connect mouse and key events to their handlers."""
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_key(self, event):
        """Delegate key press actions to helper methods."""
        key = event.key.lower()
        if key == 'y':
            self.set_cone_mode('yellow')
        elif key == 'b':
            self.set_cone_mode('blue')
        elif key == 'p':
            self.reset_initial_position()
        elif key == 't' and self.init_pos:
            self.set_target_mode()
        elif key == 'f':
            self.start_final_box()
        elif event.key == 'ctrl+z':
            self.undo_last_cone()
        self.fig.canvas.draw()

    def set_cone_mode(self, color):
        """Set mode for adding cones of a given color."""
        self.mode = color
        self.current_color = f"{color}_cone"
        title_color = "Yellow" if color == 'yellow' else "Blue"
        self.ax.set_title(f"Adding {title_color} Cones\nLeft-click to add, Right-click to finish\nCtrl+Z to undo")

    def reset_initial_position(self):
        """Reset the initial position and clear any orientation markers."""
        self.mode = 'init_pos'
        self.remove_marker('init_scatter')
        self.remove_marker('orientation_line')
        self.init_pos = None
        self.temp_target = None
        self.theta = None
        self.ax.set_title('Select Initial Position')

    def set_target_mode(self):
        """Prepare to select a target point for orientation."""
        self.mode = 'target'
        self.remove_marker('target_scatter')
        self.remove_marker('orientation_line')
        self.temp_target = None
        self.ax.set_title('Select Target Point for Orientation')

    def start_final_box(self):
        """Initialize final box mode and clear any previous box."""
        self.mode = 'final_box'
        self.final_box_points = []
        if self.final_box_plot:
            self.final_box_plot.remove()
            self.final_box_plot = None
        self.ax.set_title('Select First Point of Final Box')

    def undo_last_cone(self):
        """Remove the last added cone point and update the display."""
        if self.x_points:
            self.x_points.pop()
            self.y_points.pop()
            self.colors.pop()
            if self.scatter:
                self.scatter.remove()
                self.scatter = None
            if self.x_points:
                display_colors = ['yellow' if c == 'yellow_cone' else 'blue' for c in self.colors]
                self.scatter = self.ax.scatter(
                    self.x_points, 
                    self.y_points, 
                    c=display_colors,
                    edgecolors='black',
                    linewidth=1
                )

    def remove_marker(self, marker_name):
        """Remove a marker attribute if it exists."""
        marker = getattr(self, marker_name, None)
        if marker:
            marker.remove()
            setattr(self, marker_name, None)

    def on_click(self, event):
        """Route mouse click events based on button and current mode."""
        if event.inaxes != self.ax:
            return
        if event.button == 1:
            self.handle_left_click(event)
        elif event.button == 3:
            self.handle_right_click()
        self.fig.canvas.draw()

    def handle_left_click(self, event):
        """Process a left-click event depending on the current mode."""
        if self.mode in ['yellow', 'blue']:
            self.add_cone(event)
        elif self.mode == 'init_pos':
            self.set_initial_position(event)
        elif self.mode == 'target' and self.init_pos:
            self.set_target(event)
        elif self.mode == 'final_box':
            self.add_final_box_point(event)

    def handle_right_click(self):
        """On right-click, save data (if any) and close the plot."""
        if self.x_points or self.init_pos or self.final_box_points:
            self.save_points()
        plt.close(self.fig)

    def add_cone(self, event):
        """Add a cone point and update the scatter plot."""
        self.x_points.append(event.xdata)
        self.y_points.append(event.ydata)
        self.colors.append(self.current_color)
        if self.scatter:
            self.scatter.remove()
        display_colors = ['yellow' if c == 'yellow_cone' else 'blue' for c in self.colors]
        self.scatter = self.ax.scatter(
            self.x_points, 
            self.y_points, 
            c=display_colors,
            edgecolors='black',
            linewidth=1
        )

    def set_initial_position(self, event):
        """Set the initial position and mark it on the plot."""
        self.init_pos = (event.xdata, event.ydata)
        if self.init_scatter:
            self.init_scatter.remove()
        self.init_scatter = self.ax.scatter(
            [self.init_pos[0]], 
            [self.init_pos[1]], 
            c='green', 
            marker='*', 
            s=200, 
            label='Initial Position'
        )
        self.ax.set_title('Press F to set orientation')

    def set_target(self, event):
        """Set the target point, compute orientation, and update markers."""
        self.temp_target = (event.xdata, event.ydata)
        self.theta = self.calculate_theta(self.init_pos, self.temp_target)
        if self.target_scatter:
            self.target_scatter.remove()
        if self.orientation_line:
            self.orientation_line.remove()
        self.target_scatter = self.ax.scatter(
            [self.temp_target[0]], 
            [self.temp_target[1]], 
            c='red', 
            marker='x', 
            s=100
        )
        self.orientation_line = self.ax.plot(
            [self.init_pos[0], self.temp_target[0]], 
            [self.init_pos[1], self.temp_target[1]], 
            'r--'
        )[0]

    def add_final_box_point(self, event):
        """Collect final box points and draw the rectangle if two points are set."""
        self.final_box_points.append((event.xdata, event.ydata))
        if len(self.final_box_points) == 2:
            self.draw_final_box()

    def draw_final_box(self):
        """Draw the rectangle representing the final box."""
        if self.final_box_plot:
            self.final_box_plot.remove()
        x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
        x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
        y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
        y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
        self.final_box_plot = plt.Rectangle(
            (x1, y1), 
            x2 - x1, 
            y2 - y1, 
            fill=False, 
            color='purple', 
            linewidth=2
        )
        self.ax.add_patch(self.final_box_plot)
        self.mode = 'none'

    def calculate_theta(self, p1, p2):
        """
        Calculate the angle (in radians) between two points relative to the horizontal axis.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)

    def save_points(self):
        """
        Save the configuration (initial position, orientation, final box, and cones)
        to a text file.
        """
        filename = 'cones.txt'
        filepath = os.path.join("/home/ws/src/planning/test/integration_tests/tests/", filename)
        with open(filepath, 'w') as f:
            if self.init_pos and self.theta is not None:
                f.write(f'P {self.init_pos[0]:.1f} {self.init_pos[1]:.1f} {self.theta:.6f}\n')
            if len(self.final_box_points) == 2:
                x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
                x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
                y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
                y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
                f.write(f'F {x1:.1f} {x2:.1f} {y1:.1f} {y2:.1f}\n')
            for x, y, color in zip(self.x_points, self.y_points, self.colors):
                f.write(f'C {x:.1f} {y:.1f} {color}\n')
        print(f"Data saved to {filepath}")

def main():
    """
    Main function to display instructions and run the Track Setup Tool.
    """
    print("Track Setup Tool")
    print("1. Press 'Y' for yellow cones")
    print("2. Press 'B' for blue cones")
    print("3. Press 'P' to set initial position")
    print("4. Press 'T' to set orientation target")
    print("5. Press 'F' to set final box (2 points)")
    print("6. Left-click to add points")
    print("7. Right-click to finish and save")
    
    _ = PointTracker()
    plt.show()

if __name__ == '__main__':
    main()
