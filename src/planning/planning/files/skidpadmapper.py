import math
import numpy as np
import matplotlib.pyplot as plt

innerRadius, outerRadius = 15.25, 18.25
radius = 16.75
center_x, center_y = 16.75, 0

xPoints = []
yPoints = []

xCones = []
yCones = []

colorList = []

circleDivisions = 32

coneDivisions = 16

# =========== Cones ===========

# ========= inner ==========

for angle in np.arange(math.pi, -math.pi, -2*math.pi / coneDivisions):
    x = center_x + innerRadius * math.cos(angle)
    y = center_y + innerRadius * math.sin(angle)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#ff0000")

for angle in np.arange(0, 2*math.pi, 2*math.pi / coneDivisions):
    x = -center_x + innerRadius * math.cos(angle)
    y = center_y + innerRadius * math.sin(angle)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#0000ff")

# ========= Outer =========
for angle in np.arange(3 * math.pi / 4, -6.9 * math.pi / 8, -2*math.pi / coneDivisions):
    x = center_x + outerRadius * math.cos(angle)
    y = center_y + outerRadius * math.sin(angle)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#0000ff")

for angle in np.arange(math.pi / 4, 15 * math.pi / 8, 2*math.pi / coneDivisions):
    x = -center_x + outerRadius * math.cos(angle)
    y = center_y + outerRadius * math.sin(angle)
    xCones.append(x)
    yCones.append(y)
    colorList.append("#ff0000")

for i in range(0, 2):
    for angle in np.arange(math.pi, -math.pi, -2*math.pi / circleDivisions):
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        xPoints.append(x)
        yPoints.append(y)
        colorList.append("#00aa00")

for i in range(0, 2):
    for angle in np.arange(0, 2*math.pi, 2*math.pi / circleDivisions):
        x = -center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        xPoints.append(x)
        yPoints.append(y)
        colorList.append("#00aa00")

print(len(xPoints))
print(len(yPoints))
print(len(xCones))
print(len(yCones))
print(len(yPoints + yCones))
print(len(colorList))

f = open("skidpadpath.txt", "w")
for i in range(0, len(xPoints)):
    f.write(f"{xPoints[i]} {yPoints[i]}\n")

# Iterating through the json
# list
# for i in data['emp_details']:
# for i in range(0, len(data['x'])):
#     print(f"{data['x'][i]} {data['y'][i]} {color_encoding(data['color'][i])}")

f.close()

plt.scatter(xCones + xPoints, yCones + yPoints, s=2, c=colorList)
plt.axis('equal')
plt.show()
