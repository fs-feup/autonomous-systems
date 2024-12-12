import math

# Circle parameters
centerX = 15.0
centerY = 9.125
radius = 9.125
numPoints = 100  # Number of points to describe the circle

entryspeed=6
z = 5  # Assume constant z value
exitspeed = 7
z = "rotatingspeed"

for i in range(0,30):
    print(f"PathPoint({i/2:.3f}, 0, entryspeed),")
    
# Generate and print circular path points
for i in range(numPoints):
    theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
    x = centerX + radius * math.cos(theta)
    y = centerY + radius * math.sin(theta)
    print(f"PathPoint({x:.3f}, -{y:.3f}, {z}),")

for i in range(numPoints):
    theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
    x = centerX + radius * math.cos(theta)
    y = centerY + radius * math.sin(theta)
    print(f"PathPoint({x:.3f}, -{y:.3f}, {z}),")
    
for i in range(numPoints):
    theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
    x = centerX + radius * math.cos(theta)
    y = centerY + radius * math.sin(theta)
    print(f"PathPoint({x:.3f}, {y:.3f}, {z}),")
    
for i in range(numPoints):
    theta = (2 * math.pi / numPoints) * i - math.pi / 2  # Angle in radians, start at x=15, y=0
    x = centerX + radius * math.cos(theta)
    y = centerY + radius * math.sin(theta)
    print(f"PathPoint({x:.3f}, {y:.3f}, {z}),")


for i in range(0,20):
    print(f"PathPoint({15+i/2:.3f}, 0, exitspeed),")

for i in range(0,40):
    print(f"PathPoint({25+i/2:.3f}, 0, {0}),")