import math
import numpy as np
import matplotlib.pyplot as plt

innerRadius, outerRadius = 7.625, 10.625
radius = 9.125
center_x, center_y = 9.125, 0
startLine, endLine = -15.0, 15.0
straightPoints = 5
roundCases = 3

xPoints = []
yPoints = []

xCones = []
yCones = []

colorList = []

circleDivisions = 32

coneDivisions = 16

def shift(x):
    return x + 15.0

# =========== Cones ===========

# ========= inner ==========

for angle in np.arange(math.pi, -math.pi, -2*math.pi / coneDivisions):
    x = center_x + innerRadius * math.cos(angle)
    y = center_y + innerRadius * math.sin(angle)
    x = round(x, roundCases)
    y = round(y, roundCases)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#ff0000")

for angle in np.arange(0, 2*math.pi, 2*math.pi / coneDivisions):
    x = -center_x + innerRadius * math.cos(angle)
    y = center_y + innerRadius * math.sin(angle)
    x = round(x, roundCases)
    y = round(y, roundCases)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#0000ff")

# ========= Outer =========
for angle in np.arange(3 * math.pi / 4, -6.9 * math.pi / 8, -2*math.pi / coneDivisions):
    x = center_x + outerRadius * math.cos(angle)
    y = center_y + outerRadius * math.sin(angle)
    x = round(x, roundCases)
    y = round(y, roundCases)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#0000ff")

for angle in np.arange(math.pi / 4, 15 * math.pi / 8, 2*math.pi / coneDivisions):
    x = -center_x + outerRadius * math.cos(angle)
    y = center_y + outerRadius * math.sin(angle)
    x = round(x, roundCases)
    y = round(y, roundCases)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#ff0000")

for i in np.arange(startLine, 0, abs(startLine) // straightPoints):
    x = 0.0
    y = round(i, roundCases)
    xPoints.append(x)
    yPoints.append(y)
    colorList.append("#00aa00")

for i in range(0, 2):
    for angle in np.arange(math.pi, -math.pi, -2*math.pi / circleDivisions):
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        x = round(x, roundCases)
        y = round(y, roundCases)
        xPoints.append(x)
        yPoints.append(y)
        colorList.append("#00aa00")

for i in range(0, 2):
    for angle in np.arange(0, 2*math.pi, 2*math.pi / circleDivisions):
        x = -center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        x = round(x, roundCases)
        y = round(y, roundCases)
        xPoints.append(x)
        yPoints.append(y)
        colorList.append("#00aa00")

for i in np.arange(0, endLine + 0.01, endLine // straightPoints):
    x = 0.0
    y = round(i, roundCases)
    xPoints.append(x)
    yPoints.append(y)
    colorList.append("#00aa00")

for i in range(len(yCones)):
    yCones[i], xCones[i] = -xCones[i], shift(yCones[i])

for i in range(len(yPoints)):
    yPoints[i], xPoints[i] = -xPoints[i], shift(yPoints[i])

print(len(xPoints))
print(len(yPoints))
print(len(xCones))
print(len(yCones))
print(len(yPoints + yCones))
print(len(colorList))

f1 = open("skidpad.txt", "w")
for i in range(0, len(xPoints)):
    f1.write(f"{xPoints[i]} {yPoints[i]}\n")
f1.close()

f2 = open("skidpad_map.txt", "w")
for i in range(0, len(xCones)):
    f2.write(f"{xCones[i]} {yCones[i]}\n")
f2.close()

plt.scatter(xCones + xPoints, yCones + yPoints, s=2, c=colorList)
plt.axis('equal')
plt.show()
