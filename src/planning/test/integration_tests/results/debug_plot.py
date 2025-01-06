import matplotlib.pyplot as plt
import numpy as np
import os

def list_txt_files(folder_path):
    txt_files = [f for f in os.listdir(folder_path) if f.endswith('.txt')]
    for i, file in enumerate(txt_files, 1):
        print(f"{i}. {file}")
    return txt_files

def plot_data(filename, min_x=(-10, 10), min_y=(-10, 10)):
    path_x, path_y = [], []
    cone_x, cone_y, cone_colors = [], [], []
    
    with open(filename, 'r') as f:
        for line in f:
            data = line.strip().split()
            if data[0] == 'P':
                path_x.append(float(data[1]))
                path_y.append(float(data[2]))
            elif data[0] == 'C':
                cone_x.append(float(data[1]))
                cone_y.append(float(data[2]))
                cone_colors.append('yellow' if data[3] == 'yellow_cone' else 'blue')
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    ax.scatter(cone_x, cone_y, c=cone_colors, edgecolors='black', s=100)
    ax.scatter(path_x, path_y, c='red', s=50, label='Path')
    
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