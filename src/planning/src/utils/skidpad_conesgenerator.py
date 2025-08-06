import math
import os
import matplotlib.pyplot as plt
import numpy as np

# Circle parameters
centerX = 15.0
centerY = 9.125
radius = 7.625
outerRadius = 10.625  # Outer radius for the second circle
numPoints = 100  # Number of points to describe the circle

file_name = "./src/planning/src/utils/skidpadcones1.txt"

with open(file_name, "w") as file:
    # Generate and print circular path points
    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} {-y:.3f} {0}\n")  # Fixed: use {-y:.3f} instead of -{y:.3f}

    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + outerRadius * math.cos(theta)
        y = centerY + outerRadius * math.sin(theta)
        if abs(y)>1.5:
            file.write(f"{x:.3f} {-y:.3f} {0}\n")  # Fixed: use {-y:.3f} instead of -{y:.3f}

    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} {y:.3f} {0}\n")
        
    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + outerRadius * math.cos(theta)
        y = centerY + outerRadius * math.sin(theta)
        if abs(y)>1.5:
            file.write(f"{x:.3f} {y:.3f} {0}\n")
        
    file_path = os.path.abspath(file_name)
    print(f"File written to: {file_path}")

# Draw all the points in matplotlib
print(f"Generated {numPoints * 4} points in total.")
# create a plot to visualize the points
points = np.loadtxt(file_name)
plt.figure(figsize=(8, 8))
plt.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap='viridis')
plt.colorbar(label='Speed')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Skidpad Cones')
plt.axis('equal')
plt.grid()

# Save plot instead of showing it (for headless environments)
plt.savefig('./src/planning/src/utils/skidpad_plot.png', dpi=300, bbox_inches='tight')
print("Plot saved as skidpad_plot.png")

# Only try to show if we have a display
try:
    plt.show()
except:
    print("Display not available - plot saved to file instead")