#!/bin/bash
# Usage: ./convert_bag.sh <bag_mcap_path> <output_csv_filename>

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: ./convert_bag.sh <bag_mcap_path> <output_csv_filename>"
    echo "Example: ./convert_bag.sh ~/Downloads/acceleration.mcap acceleration_run.csv"
    exit 1
fi

# Automatically route output to the tuning_csvs folder
OUTPUT_PATH="src/invictasim/tuning_csvs/$2"

python3 src/invictasim/scripts/rosbag_to_csv.py "$1" -o "$OUTPUT_PATH"
echo "✅ Conversion complete! Saved to: $OUTPUT_PATH"
