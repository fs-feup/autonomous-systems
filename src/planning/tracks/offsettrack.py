# pylint: skip-file
# mypy: ignore-errors
"""!
Code to add outliers to a track in a file used for testing
"""

x_values = []
y_values = []
c_values = []

with open("map_250.txt", "r") as file:
    for line in file:
        line = line.strip()  # Remove leading/trailing whitespace
        if line:
            x, y, c = line.split()
            x_values.append(float(x))
            y_values.append(float(y))
            c_values.append(c)


f = open("map_250_out10.txt", "w")
count = 0

for i in range(len(x_values)):
    if i % 25 == 0:
        f.write(
            str(x_values[i] + 3) + " " + str(y_values[i]) + " " + c_values[i] + "\n"
        )
    else:
        f.write(str(x_values[i]) + " " + str(y_values[i]) + " " + c_values[i] + "\n")

f.close()
