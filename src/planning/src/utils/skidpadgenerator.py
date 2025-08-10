import math
import os
# Circle parameters
centerX = 15.0
centerY = 9.125
radius = 9.125
numPoints = 100  # Number of points to describe the circle

entryspeed = 1
exitspeed = 1
circular_speed = 1

file_name = "./src/planning/src/utils/skidpad.txt"

# Get the absolute path of the file



with open(file_name, "w") as file:

    for i in range(0,30):
        file.write(f"{i/2:.3f} 0 {entryspeed}\n")
        
    # Generate and print circular path points
    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} -{y:.3f} {circular_speed}\n")

    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} -{y:.3f} {circular_speed}\n")
        
    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} {y:.3f} {circular_speed}\n")
        
    for i in range(numPoints):
        theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
        x = centerX + radius * math.cos(theta)
        y = centerY + radius * math.sin(theta)
        file.write(f"{x:.3f} {y:.3f} {circular_speed}\n")


    for i in range(0,20):
        file.write(f"{15+i/2:.3f} 0 {exitspeed}\n")

    for i in range(0,40):
        file.write(f"{25+i/2:.3f} 0 {0}\n")
        
    file_path = os.path.abspath(file_name)
    print(f"File written to: {file_path}")