# Cloud Storage

## Package Information

### Description

This folder contains scripts for interacting with Google Cloud Storage buckets, specifically for uploading CSV files. 

### Folder Structure

The main scripts are:

- `bucket_operations.py`: Contains functions to upload, list and download files to/from a Google Cloud Storage bucket.
- `send_bucket.py`: Uploads CSV files from unit tests and evaluations to the bucket.

### Important Dependencies

- [google-cloud-storage](https://pypi.org/project/google-cloud-storage/)

## How to Run

### Install Dependencies

```sh
./dependencies_install.sh
```
### Running

To run the `send_bucket.py` script, from the workspace use the following command:

```sh
python3 src/cloud_storage/send_bucket.py <csv_file_option> <file1 file2 file3 | all>
```

 - **<csv_file_option>**: Specifies the source folder from which to read the CSV files.
 - **<file1 file2 file3 | all>**: Specifies individual file names separated by spaces, or use all to upload all files in the folder.


#### Options for **<csv_file_option>**
 - **state_est**: Reads time_exec measurements from unit tests of state estimation.
 - **perception**: Reads time_exec measurements from unit tests of perception.
 - **planning_exec_time**: Reads time_exec measurements from unit tests of planning.
 - **evaluator**: Reads measurements of each subsystem from the evaluator.
 - **planning_cone_coloring**: Reads accuracy measurements from the cone coloring unit tests.

#### Examples

Run the following commands from the workspace directory (/home/ws):

1. Upload all files from the evaluator folder:

```sh
python3 src/cloud_storage/send_bucket.py evaluator all
```

2. Upload specific files from the state estimation folder:

```sh
python3 src/cloud_storage/send_bucket.py state_est file1.csv file2.csv
```

### Extra

Make sure you have your google cloud authentication file (.json) in this directory.