import matplotlib.pyplot as plt
import os
import math
import numpy as np

class PointTracker:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.x_points = []
        self.y_points = []
        self.colors = []
        self.current_color = None
        self.scatter = None
        
        # Initial position and orientation
        self.init_pos = None
        self.temp_target = None
        self.init_scatter = None
        self.target_scatter = None
        self.orientation_line = None
        self.theta = None
        
        # Final box
        self.final_box_points = []
        self.final_box_plot = None
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.ax.set_title('Track Setup Tool\nY: Yellow Cones, B: Blue Cones\nP: Initial Position, T: Target for Orientation\nZ: Final Box Points\nRight-click to Finish')
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.grid(True)  # Enable major gridlines
        self.ax.minorticks_on()  # Enable minor ticks for finer gridlines
        self.ax.grid(which='minor', linestyle=':', linewidth=0.5)  # Customize minor gridlines
        self.ax.grid(which='major', linestyle='-', linewidth=1)  # Optional customization for major gridlines
        self.ax.set_xlim(-5, 40)
        self.ax.set_ylim(-5, 40)
        self.ax.set_autoscale_on(False)
        self.mode = 'none'
        self.fig.canvas.draw()

    def on_key(self, event):
        if event.key.lower() == 'y':
            self.mode = 'yellow'
            self.current_color = 'yellow_cone'
            self.ax.set_title('Adding Yellow Cones\nLeft-click to add, Right-click to finish')
        elif event.key.lower() == 'b':
            self.mode = 'blue'
            self.current_color = 'blue_cone'
            self.ax.set_title('Adding Blue Cones\nLeft-click to add, Right-click to finish')
        elif event.key.lower() == 'p':
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
            self.mode = 'target'
            if self.target_scatter:
                self.target_scatter.remove()
                self.target_scatter = None
            if self.orientation_line:
                self.orientation_line.remove()
                self.orientation_line = None
            self.temp_target = None
            self.ax.set_title('Select Target Point for Orientation')
        elif event.key.lower() == 'z':  # Changed from 'f' to 'z'
            self.mode = 'final_box'
            self.final_box_points = []
            if self.final_box_plot:
                self.final_box_plot.remove()
                self.final_box_plot = None
            self.ax.set_title('Select First Point of Final Box')
        
        self.fig.canvas.draw()

    def calculate_theta(self, p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        
        if event.button == 1:
            if self.mode in ['yellow', 'blue']:
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
            
            elif self.mode == 'init_pos':
                self.init_pos = (event.xdata, event.ydata)
                if self.init_scatter:
                    self.init_scatter.remove()
                self.init_scatter = self.ax.scatter([self.init_pos[0]], [self.init_pos[1]], 
                                                  c='green', marker='*', s=200, label='Initial Position')
                self.ax.set_title('Press T to set orientation')
            
            elif self.mode == 'target' and self.init_pos:
                self.temp_target = (event.xdata, event.ydata)
                self.theta = self.calculate_theta(self.init_pos, self.temp_target)
                
                if self.target_scatter:
                    self.target_scatter.remove()
                if self.orientation_line:
                    self.orientation_line.remove()
                
                self.target_scatter = self.ax.scatter([self.temp_target[0]], [self.temp_target[1]], 
                                                    c='red', marker='x', s=100)
                self.orientation_line = self.ax.plot([self.init_pos[0], self.temp_target[0]], 
                                                   [self.init_pos[1], self.temp_target[1]], 
                                                   'r--')[0]
            
            elif self.mode == 'final_box':
                self.final_box_points.append((event.xdata, event.ydata))
                if len(self.final_box_points) == 2:
                    if self.final_box_plot:
                        self.final_box_plot.remove()
                    x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
                    x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
                    y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
                    y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
                    self.final_box_plot = plt.Rectangle((x1, y1), x2-x1, y2-y1, 
                                                      fill=False, color='purple', linewidth=2)
                    self.ax.add_patch(self.final_box_plot)
                    self.mode = 'none'
            
            self.fig.canvas.draw()
        
        elif event.button == 3:
            if self.x_points or self.init_pos or self.final_box_points:
                self.save_points()
            plt.close(self.fig)

    def save_points(self):
        filename = 'cones.txt'
        with open("/home/ws/src/planning/test/integration_tests/"+filename, 'w') as f:
            # Save initial position and orientation if set
            if self.init_pos and self.theta is not None:
                f.write(f'P {self.init_pos[0]:.1f} {self.init_pos[1]:.1f} {self.theta:.6f}\n')
            
            # Save final box if set
            if len(self.final_box_points) == 2:
                x1 = min(self.final_box_points[0][0], self.final_box_points[1][0])
                x2 = max(self.final_box_points[0][0], self.final_box_points[1][0])
                y1 = min(self.final_box_points[0][1], self.final_box_points[1][1])
                y2 = max(self.final_box_points[0][1], self.final_box_points[1][1])
                f.write(f'F {x1:.1f} {x2:.1f} {y1:.1f} {y2:.1f}\n')
            
            # Save cones
            for x, y, color in zip(self.x_points, self.y_points, self.colors):
                f.write(f'C {x:.1f} {y:.1f} {color}\n')
        
        print(f"Data saved to {filename}")

def main():
    print("Track Setup Tool")
    print("1. Press 'Y' for yellow cones")
    print("2. Press 'B' for blue cones")
    print("3. Press 'P' to set initial position")
    print("4. Press 'T' to set orientation target")
    print("5. Press 'Z' to set final box (2 points)")  # Changed from F to Z
    print("6. Left-click to add points")
    print("7. Right-click to finish and save")
    
    tracker = PointTracker()
    plt.show()

if __name__ == '__main__':
    main()