from google.cloud import storage
import os

def setup_gcs_client():
    
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "src/cloud_storage/spring-ranger-426410-g8-c7c0f21fcee1.json"
    client = storage.Client()
    
    return client

def upload_csv_to_bucket(bucket_name, source_file_name, destination_blob_name):
    
    client = setup_gcs_client()
    bucket = client.bucket(bucket_name)
    blob = bucket.blob(destination_blob_name)
    blob.upload_from_filename(source_file_name)    
    print(f"File {source_file_name} uploaded to {destination_blob_name}.")

def download_csv_from_bucket(bucket_name, source_blob_name, destination_file_name):
    
    client = setup_gcs_client()
    bucket = client.bucket(bucket_name)
    blob = bucket.blob(source_blob_name)    
    blob.download_to_filename(destination_file_name)
    print(f"File {source_blob_name} downloaded to {destination_file_name}.")
