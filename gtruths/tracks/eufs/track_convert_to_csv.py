import csv
import os
from math import sqrt

def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def convert_csv(input_file):
    output_file = os.path.join(os.path.dirname(input_file), "converted_" + os.path.basename())
    with open(input_file, 'r') as csv_input, open(output_file, 'w', newline='') as csv_output:
        reader = csv.reader(csv_input)
        writer = csv.writer(csv_output)
        writer.writerow(['x', 'y', 'tag'])
        data = list(reader)[1:]  # skip header
        for row in data:
            tag = row[0]
            if tag in ["yellow", "blue"]:
                tag += "_cone"
            elif tag == "big_orange":
                x, y = float(row[1]), float(row[2])
                min_distance = float('inf')
                closest_tag = ""
                for other_row in data:
                    if other_row[0] in ["yellow", "blue"]:
                        other_x, other_y = float(other_row[1]), float(other_row[2])
                        dist = distance(x, y, other_x, other_y)
                        if dist < min_distance:
                            min_distance = dist
                            closest_tag = other_row[0] + "_cone"
                tag = closest_tag
            else:
                continue
            writer.writerow([row[1], row[2], tag])

if __name__ == "__main__":
    input_path = "track2.csv"
    print(f"Current working directory: {os.getcwd()}")
    if os.path.isfile(input_path):
        convert_csv(input_path)
    else:
        print(f"File {input_path} does not exist")
    print("Conversion completed.")
