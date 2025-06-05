import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imageio
import os
from model import bicycle_model 

L = 2.5  # Wheelbase (m)
N = 500  # Number of simulation steps
dt = 0.1  # Time step (s)

def velocity(t):
    return 5 + 2 * np.sin(0.1 * t)

def steering_angle(t):
    return np.deg2rad(10 * np.sin(0.05 * t))

frame_dir = "frames"
video_filename = "vehicle_bicycle_model.mp4"
if not os.path.exists(frame_dir):
    os.makedirs(frame_dir)

# Initial state
x, y, theta = 0.0, 0.0, 0.0
bicycle = bicycle_model()
path_x, path_y = [x], [y]
velocities = []
headings = []

for i in range(N):
    t = i * dt
    v = velocity(t)
    delta = steering_angle(t)

    bicycle.update(v, delta, t)
    x, y, theta = bicycle.x, bicycle.y, bicycle.theta

    path_x.append(x)
    path_y.append(y)
    velocities.append(v)
    headings.append(theta)

    # Visualization
    fig, axs = plt.subplots(2, 1, figsize=(8, 10))
    axs[0].set_title("Vehicle Path and Position")
    axs[0].set_xlim(-30, 100)
    axs[0].set_ylim(-20, 80)
    axs[0].set_xlabel("x (m)")
    axs[0].set_ylabel("y (m)")
    axs[0].plot(path_x, path_y, 'b-', label='Path')
    axs[0].plot(x, y, 'ro', label='Vehicle')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].set_title("Velocity and Heading over Time")
    axs[1].plot(np.arange(i+1)*dt, velocities, 'g-', label='Velocity (m/s)')
    axs[1].plot(np.arange(i+1)*dt, np.rad2deg(headings), 'r--', label='Heading (deg)')
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid(True)

    frame_path = f"{frame_dir}/frame_{i:04d}.png"
    plt.tight_layout()
    plt.savefig(frame_path)
    plt.close(fig)

print("All frames saved. Creating video...")

with imageio.get_writer(video_filename, fps=30) as writer:
    for i in range(N):
        frame_path = f"{frame_dir}/frame_{i:04d}.png"
        image = imageio.imread(frame_path)
        writer.append_data(image)

print(f"Video saved as {video_filename}")