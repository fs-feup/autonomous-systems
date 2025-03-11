import matplotlib.pyplot as plt
import numpy as np
import os

def list_txt_files(folder_path):
    txt_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.txt')])
    for i, file in enumerate(txt_files, 1):
        print(f"{i}. {file}")
    return txt_files

def parse_data_file(filename):
    """
    Read the file and extract path, cone, and car data.

    Returns:
        tuple: (path_x, path_y, cone_x, cone_y, cone_colors, car_x, car_y)
    """
    path_x, path_y = [], []
    cone_x, cone_y, cone_colors = [], [], []
    car_x, car_y = None, None

    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            key = parts[0]
            if key == 'P':
                path_x.append(float(parts[1]))
                path_y.append(float(parts[2]))
            elif key == 'C':
                cone_x.append(float(parts[1]))
                cone_y.append(float(parts[2]))
                cone_colors.append('yellow' if parts[3] == 'yellow_cone' else 'blue')
            elif key == 'V':
                car_x = float(parts[1])
                car_y = float(parts[2])
    return path_x, path_y, cone_x, cone_y, cone_colors, car_x, car_y

def compute_limits(all_x, all_y, default_x=(-10, 10), default_y=(-10, 10)):
    """
    Compute axis limits given all data points and default limits.

    Returns:
        tuple: (x_min, x_max, y_min, y_max)
    """
    if all_x:
        x_min = min(default_x[0], min(all_x))
        x_max = max(default_x[1], max(all_x))
    else:
        x_min, x_max = default_x
    if all_y:
        y_min = min(default_y[0], min(all_y))
        y_max = max(default_y[1], max(all_y))
    else:
        y_min, y_max = default_y
    return x_min, x_max, y_min, y_max

def plot_parsed_data(ax, path_x, path_y, cone_x, cone_y, cone_colors, car_x, car_y, min_x, min_y):
    """
    Plot the parsed data on the provided axes.
    """
    # Plot cones.
    ax.scatter(cone_x, cone_y, c=cone_colors, edgecolors='black', s=100)
    
    # Plot path data.
    if path_x and path_y:
        ax.plot(path_x, path_y, 'r-', linewidth=1, zorder=1)
        if len(path_x) > 1:
            ax.scatter(path_x[1:], path_y[1:], c='red', s=50, label='Path', zorder=2)
        ax.scatter(path_x[0], path_y[0], c='orange', s=50, label='Start', zorder=3)
    
    # Plot car position if available.
    if car_x is not None and car_y is not None:
        ax.scatter(car_x, car_y, c='green', s=150, label='Car Position', zorder=4)
    
    # Compute and set axis limits.
    all_x = path_x + cone_x
    all_y = path_y + cone_y
    x_min, x_max, y_min, y_max = compute_limits(all_x, all_y, min_x, min_y)
    ax.set_xlim(x_min - 5, x_max + 5)
    ax.set_ylim(y_min - 5, y_max + 5)
    
    # Set labels and grid.
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Track Visualization')

def plot_data(filename, min_x=(-10, 10), min_y=(-10, 10)):
    """
    Parse the file and plot the track, cones, and car position.
    
    Parameters:
        filename (str): Path to the data file.
        min_x (tuple): Default x-axis limits.
        min_y (tuple): Default y-axis limits.
    """
    path_x, path_y, cone_x, cone_y, cone_colors, car_x, car_y = parse_data_file(filename)
    _, ax = plt.subplots(figsize=(10, 8))
    plot_parsed_data(ax, path_x, path_y, cone_x, cone_y, cone_colors, car_x, car_y, min_x, min_y)
    plt.show()

def main():
    folder_path = "/home/ws/src/planning/test/integration_tests/results/"
    txt_files = list_txt_files(folder_path)
    
    while True:
        try:
            choice = int(input("\nEnter the number of the file to plot (0 to exit): "))
            if choice == 0:
                break
            if 1 <= choice <= len(txt_files):
                selected_file = os.path.join(folder_path, txt_files[choice-1])
                plot_data(selected_file, min_x=(-5, 20), min_y=(-5, 30))
            else:
                print("Invalid selection. Please try again.")
        except ValueError:
            print("Please enter a valid number.")

if __name__ == "__main__":
    main()