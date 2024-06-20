import os
import sys
import argparse
from bucket_operations import upload_csv_to_bucket

FOLDERS = {
    'state_est': 'performance/exec_time/ekf_state_est',
    'perception': 'performance/exec_time/perception',
    'planning': 'performance/exec_time/planning',
    'evaluator': 'performance/evaluator_metrics',
}
def list_files(directory, filenames):
    try:
        all_files = os.listdir(directory)
    except FileNotFoundError:
        print(f"Error: Directory '{directory}' not found.")
        return []

    if filenames == 'all':
        return [file for file in all_files if file not in ['.gitkeep', '.gitignore']]

    selected_files = []
    for filename in filenames:
        if filename in all_files:
            selected_files.append(filename)
        else:
            print(f"Warning: File '{filename}' not found in directory '{directory}'.")

    return selected_files

def main():
    parser = argparse.ArgumentParser(description="Retrieve files from specified folders.")
    parser.add_argument('folder', choices=FOLDERS.keys(), help="Folder to retrieve files from.")
    parser.add_argument('filenames', nargs='+', help="Names of the files to retrieve, or 'all' to get all files.")
    args = parser.parse_args()

    directory = FOLDERS[args.folder]
    filenames = args.filenames if args.filenames[0] != 'all' else 'all'

    files = list_files(directory, filenames)

    if files:
        print("Retrieved files:")
        for file in files:
            source_file_name = directory + "/" + file
            destination_path = f"{args.folder}/{file}"
            upload_csv_to_bucket("test_eval", source_file_name, destination_path)
    else:
        print("No files retrieved.")

if __name__ == "__main__":
    main()