import math
import os

# Geometry
CENTER_X = 15.0
CENTER_Y = 9.125
RADIUS = 9.0
POINTS_PER_CIRCLE = 100
LANE_SPACING = 1.0      # distance between points on the straights (m)
RUNOFF_DISTANCE = 40  # straight after the circles, where the car brakes and stops (m)

ENTRY_POINTS = int(round(CENTER_X / LANE_SPACING))
EXIT_POINTS = int(round(RUNOFF_DISTANCE / LANE_SPACING)) + 1

# Speed profile
MINIMUM_SPEED = 2.0     # m/s, the controller needs a non-zero error to pull away
CIRCULAR_SPEED = 9.0    # m/s, held around the circles
ACCELERATION = 4.0      # m/s^2, used to build up to CIRCULAR_SPEED
BRAKING_DISTANCE = 16.0  # m, from CIRCULAR_SPEED to a standstill at the end

FILE_NAME = "./src/planning/src/utils/skidpad.txt"


def build_points():
    """Entry straight, four circles (two right then two left), exit straight."""
    points = [(i * LANE_SPACING, 0.0) for i in range(ENTRY_POINTS)]

    for sign in (-1.0, -1.0, 1.0, 1.0):
        for i in range(POINTS_PER_CIRCLE):
            theta = (2 * math.pi / POINTS_PER_CIRCLE) * i - math.pi / 2
            points.append((CENTER_X + RADIUS * math.cos(theta),
                           sign * (CENTER_Y + RADIUS * math.sin(theta))))

    points += [(CENTER_X + i * LANE_SPACING, 0.0) for i in range(EXIT_POINTS)]
    return points


def build_speeds(points):
    """Accelerate to CIRCULAR_SPEED, hold it, then brake to a stop as the last circle ends."""
    steps = [math.dist(points[i - 1], points[i]) for i in range(1, len(points))]
    last_circle = ENTRY_POINTS + 4 * POINTS_PER_CIRCLE - 1

    # Start at the minimum speed and build up at ACCELERATION, capped at the cornering speed.
    speeds = [MINIMUM_SPEED]
    for step in steps[:last_circle]:
        speeds.append(min(math.sqrt(speeds[-1] ** 2 + 2 * ACCELERATION * step), CIRCULAR_SPEED))

    # Braking starts where the last circle ends; everything past the stop stays at zero.
    deceleration = CIRCULAR_SPEED ** 2 / (2 * BRAKING_DISTANCE)
    for step in steps[last_circle:]:
        remaining = speeds[-1] ** 2 - 2 * deceleration * step
        speeds.append(math.sqrt(remaining) if remaining > 0.0 else 0.0)

    return speeds


points = build_points()
speeds = build_speeds(points)

with open(FILE_NAME, "w") as file:
    for (x, y), speed in zip(points, speeds):
        file.write(f"{x:.3f} {y:.3f} {speed:.3f}\n")

print(f"File written to: {os.path.abspath(FILE_NAME)}")
