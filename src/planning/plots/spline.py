# pylint: skip-file
# mypy: ignore-errors
import matplotlib.pyplot as plt

x_values0 = []
y_values0 = []
ox_values0 = []
oy_values0 = []
rx_values0 = []
ry_values0 = []

x_values1 = []
y_values1 = []
ox_values1 = []
oy_values1 = []
rx_values1 = []
ry_values1 = []

with open("spline0.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            x_values0.append(float(x))
            y_values0.append(float(y))

with open("spline1.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            x_values1.append(float(x))
            y_values1.append(float(y))

with open("deletedoutliers0.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            ox_values0.append(float(x))
            oy_values0.append(float(y))

with open("deletedoutliers1.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            ox_values1.append(float(x))
            oy_values1.append(float(y))

with open("../tracks/outlier_test1.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y, c = line.split()
            if c == "yellow_cone":
                rx_values0.append(float(x))
                ry_values0.append(float(y))
            elif c == "blue_cone":
                rx_values1.append(float(x))
                ry_values1.append(float(y))


plt.plot(ox_values0, oy_values0, "g-", label="Corrected Track Limits")
plt.plot(ox_values1, oy_values1, "g-")

plt.plot(x_values0, y_values0, "r-", label="Spline Approximation")
plt.plot(x_values1, y_values1, "r-")


plt.plot(rx_values0, ry_values0, "bo", label="Uncorrected Blue Cones", markersize=5)
plt.plot(rx_values1, ry_values1, "yo", label="Uncorrected Yellow Cones", markersize=5)

plt.plot(axis="equal")
plt.legend()
plt.show()
