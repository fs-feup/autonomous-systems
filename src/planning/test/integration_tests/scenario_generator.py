import matplotlib.pyplot as plt
import os
import math
import numpy as np

class PointTracker:
    """
    A class to interactively track and mark points on a matplotlib plot.
    
    This tool allows the user to mark different types of points by clicking on the plot:
      - Yellow and Blue cones (for marking positions on a track).
      - The initial position of an object.
      - A target point to define the orientation from the initial position.
      - Two points that define a rectangular "final box".
      
    The points and configuration data are saved to a file when the user finishes the setup.
    """

    def __init__(self):
        # Create the main figure and axes for plotting
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        
        # Lists to store cone coordinates and their respective colors
        self.x_points = []
        self.y_points = []
        self.colors = []
        
        # Variable to store the currently selected cone color (yellow or blue)
        self.current_color = None
        self.scatter = None  # Reference to the scatter plot of cones
        
        # Variables for initial position and orientation target
        self.init_pos = None  # Tuple to store the initial position (x, y)
        self.temp_target = None  # Temporary target point used to compute orientation
        self.init_scatter = None  # Scatter plot marker for the initial position
        self.target_scatter = None  # Marker for the target point for orientation
        self.orientation_line = None  # Line indicating the orientation from the initial position to the target
        self.theta = None  # Orientation angle computed from initial position to target
        
        # Variables for defining the final box area (two points)
        self.final_box_points = []
        self.final_box_plot = None
        
        
        # Block default key handlers
        self.fig.canvas.mpl_disconnect(self.fig.canvas.manager.key_press_handler_id)
    
        
        # Connect mouse and key events to their respective handlers
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        # Configure the plot: title, labels, gridlines, and axis limits
        self.ax.set_title('Track Setup Tool\nY: Yellow Cones, B: Blue Cones\nP: Initial Position, T: Target for Orientation\nZ: Final Box Points\nRight-click to Finish')
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.grid(True)  # Enable major gridlines
        self.ax.minorticks_on()  # Enable minor ticks for finer gridlines
        self.ax.grid(which='minor', linestyle=':', linewidth=0.5)  # Customize minor gridlines
        self.ax.grid(which='major', linestyle='-', linewidth=1)  # Customize major gridlines
        self.ax.set_xlim(-5, 40)
        self.ax.set_ylim(-5, 40)
        self.ax.set_autoscale_on(False)
        
        # Mode variable indicates the current type of point being added:
        # 'yellow', 'blue', 'init_pos', 'target', 'final_box', or 'none'
        self.mode = 'none'
        self.fig.canvas.draw()

    def on_key(self, event):
        """
        Handle key press events to set the mode for adding points.
        
        Keys and their functionalities:
            - 'y' or 'Y': Enter yellow cone mode.
            - 'b' or 'B': Enter blue cone mode.
            - 'p' or 'P': Set the initial position.
            - 't' or 'T': Set the target point for orientation (requires an initial position to be set).
            - 'F' or 'F': Define the final box (select 2 points).
            - 'ctrl+z': Undo the last added cone point.
        """
        if event.key.lower() == 'y':
            self.mode = 'yellow'
            self.current_color = 'yellow_cone'
            self.ax.set_title('Adding Yellow Cones\nLeft-click to add, Right-click to finish\nCtrl+Z to undo')
        elif event.key.lower() == 'b':
            self.mode = 'blue'
            self.current_color = 'blue_cone'
            self.ax.set_title('Adding Blue Cones\nLeft-click to add, Right-click to finish\nCtrl+Z to undo')
        elif event.key.lower() == 'p':
            # Reset the initial position and orientation markers
            self.mode = 'init_pos'
            if self.init_scatter:
                self.init_scatter.remove()
                self.init_scatter = None
            if self.orientation_line:
                self.orientation_line.remove()
                self.orientation_line = None
            self.init_pos = None
            self.temp_target = None
            self.theta = None
            self.ax.set_title('Select Initial Position')
        elif event.key.lower() == 't' and self.init_pos:
            # Set the mode to add a target point to define orientation
            self.mode = 'target'
            if self.target_scatter:
                self.target_scatter.remove()
                self.target_scatter = None
            if self.orientation_line:
                self.orientation_line.remove()
                self.orientation_line = None
            self.temp_target = None
            self.ax.set_title('Select Target Point for Orientation')
        elif event.key.lower() == 'f':
            # Reset final box points and set mode to select final box points
            self.mode = 'final_box'
            self.final_box_points = []
            if self.final_box_plot:
                self.final_box_plot.remove()
                self.final_box_plot = None
            self.ax.set_title('Select First Point of Final Box')
        elif event.key == 'ctrl+z':  # Handle Ctrl+Z to undo the last added cone
            if len(self.x_points) > 0:
                self.x_points.pop()
                self.y_points.pop()
                self.colors.pop()
                
                if self.scatter:
                    self.scatter.remove()
                    self.scatter = None  # Set to None after removal
                
                # Re-plot remaining cone points if any exist
                if len(self.x_points) > 0:
                    display_colors = ['yellow' if c == 'yellow_cone' else 'blue' for c in self.colors]
                    self.scatter = self.ax.scatter(
                        self.x_points, 
                        self.y_points, 
                        c=display_colors,
                        edgecolors='black',
                        linewidth=1
                    )
        
        # Update the canvas after handling the key event
        self.fig.canvas.draw()

    def calculate_theta(self, p1, p2):
        """
        Calculate the angle (in radians) between two points and the horizontal axis.
        
        Parameters:
            p1 (tuple): The initial point (x, y).
            p2 (tuple): The target point (x, y) used for determining orientation.
        
        Returns:
            float: The angle in radians computed using atan2.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)

    def on_click(self, event):
        """
        Handle mouse click events to add points based on the current mode.
        
        Left-click (button==1) actions:
            - In 'yellow' or 'blue' mode: Adds a cone point with the corresponding color.
            - In 'init_pos' mode: Sets the initial position.
            - In 'target' mode: Sets the target point for orientation, draws a line from the initial position.
            - In 'final_box' mode: Collects two points to define a rectangular box.
        
        Right-click (button==3) action:
            - If any points are set, saves all data to a file and closes the plot.
        """
        # Ensure the click is within the axes area
        if event.inaxes != self.ax:
            return
        
        if event.button == 1:  # Left-click actions
            if self.mode in ['yellow', 'blue']:
                # Add a cone point with the current color
                self.x_points.append(event.xdata)
                self.y_points.append(event.ydata)
                self.colors.append(self.current_color)
                
                # Remove the previous scatter plot (if it exists) to update it
                if self.scatter:
                    self.scatter.remove()
                
                # Determine display colors: use actual color names for plotting
                display_colors = ['yellow' if c == 'yellow_cone' else 'blue' for c in self.colors]
                self.scatter = self.ax.scatter(
                    self.x_points, 
                    self.y_points, 
                    c=display_colors,
                    edgecolors='black',
                    linewidth=1
                )
            
            elif self.mode == 'init_pos':
                # Set the initial position based on the clicked coordinates
                self.init_pos = (event.xdata, event.ydata)
                if self.init_scatter:
                    self.init_scatter.remove()
                # Mark the initial position with a green star
                self.init_scatter = self.ax.scatter(
                    [self.init_pos[0]], 
                    [self.init_pos[1]], 
                    c='green', 
                    marker='*', 
                    s=200, 
                    label='Initial Position'
                )
                self.ax.set_title('Press F to set orientation')
            
            elif self.mode == 'target' and self.init_pos:
                # Set the target point to determine orientation
                self.temp_target = (event.xdata, event.ydata)
                # Calculate orientation angle between initial position and target
                self.theta = self.calculate_theta(self.init_pos, self.temp_target)
                
                # Update markers: remove previous target and orientation line if they exist
                if self.target_scatter:
                    self.target_scatter.remove()
                if self.orientation_line:
                    self.orientation_line.remove()
                
                # Mark the target point with a red "x" and draw a dashed line to show orientation
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
            
            elif self.mode == 'final_box':
                # Collect two points to define the final rectangular box
                self.final_box_points.append((event.xdata, event.ydata))
                if len(self.final_box_points) == 2:
                    # Once two points are set, determine the lower left corner and dimensions
                    if self.final_box_plot:
                        self.final_box_plot.remove()
                    x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
                    x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
                    y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
                    y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
                    # Create a rectangle (without fill) to represent the final box area
                    self.final_box_plot = plt.Rectangle(
                        (x1, y1), 
                        x2 - x1, 
                        y2 - y1, 
                        fill=False, 
                        color='purple', 
                        linewidth=2
                    )
                    self.ax.add_patch(self.final_box_plot)
                    # Exit the final box mode
                    self.mode = 'none'
            
            # Refresh the canvas after a left-click event
            self.fig.canvas.draw()
        
        elif event.button == 3:  # Right-click actions: finish and save data
            if self.x_points or self.init_pos or self.final_box_points:
                self.save_points()
            plt.close(self.fig)

    def save_points(self):
        """
        Save the configuration data (cones, initial position, orientation, and final box)
        to a text file under the tests directory.
        
        The file 'cones.txt' is written with the following format:
          - P <x> <y> <theta>: initial position and orientation (if set)
          - F <x1> <x2> <y1> <y2>: coordinates defining the final box (if two points were selected)
          - C <x> <y> <color>: each cone point, with the cone color ('yellow_cone' or 'blue_cone')
        """
        filename = 'cones.txt'
        filepath = "/home/ws/src/planning/test/integration_tests/tests/" + filename
        
        with open(filepath, 'w') as f:
            # Save initial position and orientation if available
            if self.init_pos and self.theta is not None:
                f.write(f'P {self.init_pos[0]:.1f} {self.init_pos[1]:.1f} {self.theta:.6f}\n')
            
            # Save final box coordinates if two points were defined
            if len(self.final_box_points) == 2:
                x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
                x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
                y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
                y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
                f.write(f'F {x1:.1f} {x2:.1f} {y1:.1f} {y2:.1f}\n')
            
            # Save all cone points with their colors
            for x, y, color in zip(self.x_points, self.y_points, self.colors):
                f.write(f'C {x:.1f} {y:.1f} {color}\n')
        
        print(f"Data saved to {filepath}")

def main():
    """
    Main function to provide instructions and run the Track Setup Tool.
    
    Instructions:
      1. Press 'Y' to switch to yellow cone mode.
      2. Press 'B' to switch to blue cone mode.
      3. Press 'P' to set the initial position.
      4. Press 'T' to set the orientation target (after setting the initial position).
      5. Press 'F' to define the final box area (select 2 points).
      6. Use left-click to add points.
      7. Use right-click to finish, save the data, and close the plot.
    """
    print("Track Setup Tool")
    print("1. Press 'Y' for yellow cones")
    print("2. Press 'B' for blue cones")
    print("3. Press 'P' to set initial position")
    print("4. Press 'T' to set orientation target")
    print("5. Press 'F' to set final box (2 points)")  # Changed from F to Z
    print("6. Left-click to add points")
    print("7. Right-click to finish and save")
    
    # Create an instance of PointTracker and display the plot
    tracker = PointTracker()
    plt.show()

if __name__ == '__main__':
    main()
