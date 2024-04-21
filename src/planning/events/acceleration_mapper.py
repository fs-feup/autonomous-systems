# pylint: skip-file
# mypy: ignore-errors
import numpy as np
import matplotlib.pyplot as plt

# Define track parameters
startLine, endLine = -0.3, 100.0
straightPoints = 30
roundCases = 3

# Initialize lists to store coordinates and colors
xPoints = []
yPoints = []

xCones = []
yCones = []

colorList = []

# Generate blue cones on the left side of the track
for i in np.arange(0, 75.01, 5):
    xCones.append(-1.5)
    yCones.append(i)
    colorList.append("#0000ff")

# Generate red cones on the right side of the track
for i in np.arange(0, 75.01, 5):
    xCones.append(1.5)
    yCones.append(i)
    colorList.append("#ff0000")

# Generate points on the straight section of the track
for i in np.arange(startLine, endLine, abs(endLine - startLine) // straightPoints):
    x = 0.0
    y = round(i, roundCases)
    xPoints.append(x)
    yPoints.append(y)
    colorList.append("#00aa00")

# Print lengths of lists for debugging
print(len(xPoints))
print(len(yPoints))
print(len(xCones))
print(len(yCones))
print(len(yPoints + yCones))
print(len(colorList))

# Reflect cone coordinates across the y-axis
for i in range(len(yCones)):
    yCones[i], xCones[i] = -xCones[i], yCones[i]

# Reflect point coordinates across the y-axis
for i in range(len(yPoints)):
    yPoints[i], xPoints[i] = -xPoints[i], yPoints[i]

# Write point coordinates to a file
f1 = open("acceleration.txt", "w")
for i in range(0, len(xPoints)):
    f1.write(f"{xPoints[i]} {yPoints[i]}\n")
f1.close()

# Write cone coordinates to a file
f2 = open("acceleration_map.txt", "w")
for i in range(0, len(xCones)):
    f2.write(f"{xCones[i]} {yCones[i]}\n")
f2.close()

# Visualize cones and points using scatter plot
plt.scatter(xCones + xPoints, yCones + yPoints, s=2, c=colorList)
plt.axis("equal")
plt.show()
