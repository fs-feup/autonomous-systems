import os
import sys
import argparse
from bucket_operations import upload_csv_to_bucket

# Define the folders to retrieve files from
FOLDERS = {
    "state_est": "performance/exec_time/ekf_state_est",
    "perception": "performance/exec_time/perception",
    "planning_exec_time": "performance/exec_time/planning",
    "evaluator": "performance/evaluator_metrics",
    "power_log": "performance/power_log_metrics",
    "planning_cone_coloring": "performance/other_metrics/planning",
}


def list_files(directory, filenames):
    """!
    List files in a directory.

    Args:
        directory: Directory to list files from.
        filenames: Names of the files to retrieve, or 'all' to get all files.
    """
    try:
        all_files = os.listdir(directory)
    except FileNotFoundError:
        print(f"Error: Directory '{directory}' not found.")
        return []

    if filenames == "all":
        return [file for file in all_files if file not in [".gitkeep", ".gitignore"]]

    selected_files = []
    for filename in filenames:
        if filename in all_files:
            selected_files.append(filename)
        else:
            print(f"Warning: File '{filename}' not found in directory '{directory}'.")

    return selected_files


def main():
    """!
    Retrieve files from specified folders. Using argparse to parse command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Retrieve files from specified folders."
    )
    parser.add_argument(
        "folder", choices=FOLDERS.keys(), help="Folder to retrieve files from."
    )
    parser.add_argument(
        "filenames",
        nargs="+",
        help="Names of the files to retrieve, or 'all' to get all files.",
    )
    args = parser.parse_args()

    directory = FOLDERS[args.folder]
    filenames = args.filenames if args.filenames[0] != "all" else "all"

    files = list_files(directory, filenames)

    if files:
        print("Retrieved files:")
        for file in files:
            source_file_name = directory + "/" + file
            destination_path = f"{args.folder}/{file}"
            upload_csv_to_bucket("as_evaluation", source_file_name, destination_path)
    else:
        print("No files retrieved.")


if __name__ == "__main__":
    main()
