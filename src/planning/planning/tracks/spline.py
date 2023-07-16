import matplotlib.pyplot as plt

x_values = []
y_values = []
ox_values = []
oy_values = []
rx_values = []
ry_values = []

with open("spline.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            x_values.append(float(x))
            y_values.append(float(y))

with open("deletedoutliers.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y = line.split()
            ox_values.append(float(x))
            oy_values.append(float(y))

with open("map_mock.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y, c = line.split()
            if c == "yellow_cone":
                rx_values.append(float(x))
                ry_values.append(float(y))


plt.plot(x_values, y_values, 'r-')
plt.plot(ox_values, oy_values, 'g-')
plt.plot(rx_values, ry_values, 'b-')
plt.plot(axis='equal')
plt.show()