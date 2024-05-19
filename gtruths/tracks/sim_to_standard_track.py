"""
To transform the original sim track into the standardized track.csv file, create a folder inside the 'tracks/<sim>' folder with the same name as the track (except for the .csv) where
you should place the original .csv file with the track inside that folder with the name "orig_<trackname>". If this folder does not exist you won't be able to save the points to the desired file. 
The tracl will be automatically exported to a .csv with the FSDS pattern file.

Make sure you ran the 'dependencies_install.sh' file to install matplotlib and pandas.

You should select which track to run this file with by changing the name of the variable immediatly below.
"""

import matplotlib.pyplot as plt
import pandas as pd
import os
import csv
import yaml
import argparse

def read_pacsim(filename):   
    with open(filename, 'r') as file:
        try:
            initial_points = []
            data = yaml.safe_load(file) 
            track_data = data['track']
            point_list = track_data['left']
            for item in point_list:            
                x = item['position'][0]
                y = item['position'][1]
                color = item['class']
                initial_points.append((float(x), float(y), color))
            return initial_points

        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")
            return None

def read_eufs(filename):
    initial_points = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header if present
        for row in reader:
            color, x, y = row[:3]  # Assuming tag is in the third column
            initial_points.append((float(x), float(y), color))
    return initial_points

def export_to_csv(points, filename):
    df = pd.DataFrame(points, columns=['x', 'y', 'color'])  # Export only the added points
    df.to_csv(filename, index=False)

def main():
    parser = argparse.ArgumentParser(description="Parse track data and export to CSV.")
    parser.add_argument('--sim', type=str, default='eufs', help='Simulator type (fsds, eufs, pacsim)')
    parser.add_argument('--track_name', type=str, default='small_track', help='Name of the track in file')
    
    args = parser.parse_args()
    sim = args.sim
    track_name = args.track_name

    file_extension = ""
    if sim == "eufs":
        file_extension = ".csv"
    elif sim == "pacsim":
        file_extension = ".yaml"
    
    # Get the current directory
    current_directory = os.getcwd()
    
    # Concatenate current directory with the provided filename
    filename = os.path.join(current_directory, 'tracks', sim, track_name, f'orig_{track_name}{file_extension}')

    initial_points = []
    if sim == "eufs":
        initial_points = read_eufs(filename)
    elif sim == "pacsim":
        initial_points = read_pacsim(filename)
    
    final_file_name = os.path.join('tracks', sim, track_name, f'{track_name}.csv')
    export_to_csv(initial_points, final_file_name)

if __name__ == "__main__":
    main()
