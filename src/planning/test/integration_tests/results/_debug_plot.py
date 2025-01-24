import matplotlib.pyplot as plt
import numpy as np
import os

def list_txt_files(folder_path):
    txt_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.txt')])
    for i, file in enumerate(txt_files, 1):
        print(f"{i}. {file}")
    return txt_files

def plot_data(filename, min_x=(-10, 10), min_y=(-10, 10)):
    path_x, path_y = [], []
    cone_x, cone_y, cone_colors = [], [], []
    car_x, car_y = None, None
    
    with open(filename, 'r') as f:
        for line in f:
            data = line.strip().split()
            if len(data) == 0:
                continue
            if data[0] == 'P':
                path_x.append(float(data[1]))
                path_y.append(float(data[2]))
            elif data[0] == 'C':
                cone_x.append(float(data[1]))
                cone_y.append(float(data[2]))
                cone_colors.append('yellow' if data[3] == 'yellow_cone' else 'blue')
            elif data[0] == 'V':
                car_x = float(data[1])
                car_y = float(data[2])
    
    fig, ax = plt.subplots(figsize=(11, 10))
    
    ax.scatter(cone_x, cone_y, c=cone_colors, edgecolors='black', s=100)
    
    ax.plot(path_x[0:], path_y[0:], 'r-', linewidth=1, zorder=1)
    ax.scatter(path_x[1:], path_y[1:], c='red', s=50, label='Path', zorder=2)
    ax.scatter(path_x[0], path_y[0], c='orange', s=50, label='Start', zorder=3)
    
    if car_x is not None and car_y is not None:
        ax.scatter(car_x, car_y, c='green', s=150, label='Car Position', zorder=4)
        
    all_x = path_x + cone_x
    all_y = path_y + cone_y
    
    x_min = min(min_x[0], min(all_x)) if all_x else min_x[0]
    x_max = max(min_x[1], max(all_x)) if all_x else min_x[1]
    y_min = min(min_y[0], min(all_y)) if all_y else min_y[0]
    y_max = max(min_y[1], max(all_y)) if all_y else min_y[1]
    
    ax.set_xlim(x_min - 5, x_max + 5)
    ax.set_ylim(y_min - 5, y_max + 5)
    
    ax.grid(True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Track Visualization')
    plt.show()

def main():
    folder_path = "/home/ws/src/planning/test/integration_tests/results/"
    txt_files = list_txt_files(folder_path)
    
    # open all the files on the first run
    for i in range(1, len(txt_files)+1):
        selected_file = os.path.join(folder_path, txt_files[i-1])
        plot_data(selected_file, min_x=(-2, 20), min_y=(-2, 20))
    
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