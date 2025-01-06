import matplotlib.pyplot as plt
import os

class PointTracker:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.x_points = []
        self.y_points = []
        self.colors = []
        self.current_color = None
        self.scatter = None
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.ax.set_title('Cone Tracking Tool\nPress Y for Yellow Cones, B for Blue Cones\nRight-click to Finish')
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.grid(True)
        self.ax.set_xlim(-10, 40)
        self.ax.set_ylim(-10, 50)
        self.ax.set_autoscale_on(False)
        self.fig.canvas.draw()

    def on_key(self, event):
        if event.key.lower() == 'y':
            self.current_color = 'yellow_cone'
            self.ax.set_title('Adding Yellow Cones\nLeft-click to add, Right-click to finish')
            self.fig.canvas.draw()
        elif event.key.lower() == 'b':
            self.current_color = 'blue_cone'
            self.ax.set_title('Adding Blue Cones\nLeft-click to add, Right-click to finish')
            self.fig.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax or self.current_color is None:
            return
        
        if event.button == 1:
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
            
            self.fig.canvas.draw()
        
        elif event.button == 3:
            if self.x_points:
                self.save_points()
            plt.close(self.fig)

    def save_points(self):
        filename = 'cones.txt'
        with open("/home/ws/src/planning/test/integration_tests/"+filename, 'w') as f:
            for x, y, color in zip(self.x_points, self.y_points, self.colors):
                f.write(f'C {x:.1f} {y:.1f} {color}\n')
        
        print(f"Cones saved to {filename}")

def main():
    print("Cone Tracking Tool")
    print("1. Press 'Y' for yellow cones")
    print("2. Press 'B' for blue cones")
    print("3. Left-click to add cones")
    print("4. Right-click to finish")
    
    tracker = PointTracker()
    plt.show()

if __name__ == '__main__':
    main()