from google.cloud import storage
import os


def setup_gcs_client():
    """!
    Set up the Google Cloud Storage client.
    """
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = (
        "src/cloud_storage/evaluation-data-427214-926daf8b7126.json"
    )
    client = storage.Client()

    return client


def upload_csv_to_bucket(bucket_name, source_file_name, destination_blob_name):
    """!
    Upload a CSV file to a Google Cloud Storage bucket.

    Args:
        bucket_name: Name of the bucket to upload the file to.
        source_file_name: Path to the file to upload.
        destination_blob_name: Name of the file in the bucket.
    """

    client = setup_gcs_client()
    bucket = client.bucket(bucket_name)
    blob = bucket.blob(destination_blob_name)
    blob.upload_from_filename(source_file_name)
    print(f"File {source_file_name} uploaded to {destination_blob_name}.")


def download_csv_from_bucket_to_folder(
    bucket_name, source_blob_name, destination_folder, file_name
):
    """!
    Download a CSV file from a Google Cloud Storage bucket to a specified folder.

    Args:
        bucket_name: Name of the bucket to download the file from.
        source_blob_name: Name of the file in the bucket.
        destination_folder: Folder to save the file to.
        file_name: Name of the file to save.
    """
    client = setup_gcs_client()
    bucket = client.bucket(bucket_name)
    blob = bucket.blob(source_blob_name)

    # Ensure the destination folder and any necessary subdirectories exist
    destination_file_path = os.path.join(destination_folder, file_name)
    os.makedirs(os.path.dirname(destination_file_path), exist_ok=True)

    blob.download_to_filename(destination_file_path)
    print(f"File {source_blob_name} downloaded to {destination_file_path}.")


def list_blobs(bucket_name):
    """!
    List all files in a Google Cloud Storage bucket.
    Args:
        bucket_name: Name of the bucket to list files from.
    """
    client = setup_gcs_client()
    blobs = client.list_blobs(bucket_name)

    return [blob.name for blob in blobs]
