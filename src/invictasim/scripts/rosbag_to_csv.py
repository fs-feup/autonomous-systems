#!/usr/bin/env python3
import os
import sys
import argparse
import pandas as pd
import numpy as np

# ROS 2 deserialization
import rosbag2_py
from rclpy.serialization import deserialize_message

# Custom and standard messages
from custom_interfaces.msg import ControlCommand, Pose, Velocities, WheelRPM

def get_true_time(msg, bag_timestamp_ns):
    """Attempts to get the hardware creation time from the header. Falls back to bag time."""
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        t_ns = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec
        if t_ns > 0:
            return t_ns
    return bag_timestamp_ns

def read_bag_data(bag_path):
    print(f"\nReading ROS2 bag: {bag_path}")
    
    # Setup bag reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        sys.exit(1)
    
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    
    data = {
        "control": [],
        "pose": [],
        "velocities": [],
        "fl_rpm": [],
        "fr_rpm": [],
        "rl_rpm": [],
        "rr_rpm": [],
        "motor_rpm": []
    }
    
    while reader.has_next():
        topic, data_serialized, bag_t_ns = reader.read_next()
        if topic not in topic_types:
            continue
            
        if topic == "/control/command":
            msg = deserialize_message(data_serialized, ControlCommand)
            t = get_true_time(msg, bag_t_ns)
            data["control"].append((t, msg.throttle_fl, msg.throttle_fr, msg.throttle_rl, msg.throttle_rr, msg.steering))
            
        elif topic == "/state_estimation/vehicle_pose":
            msg = deserialize_message(data_serialized, Pose)
            t = get_true_time(msg, bag_t_ns)
            data["pose"].append((t, msg.x, msg.y, msg.theta))
            
        elif topic == "/state_estimation/velocities":
            msg = deserialize_message(data_serialized, Velocities)
            t = get_true_time(msg, bag_t_ns)
            data["velocities"].append((t, msg.velocity_x, msg.velocity_y, msg.angular_velocity))
            
        elif topic == "/vehicle/fl_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            t = get_true_time(msg, bag_t_ns)
            data["fl_rpm"].append((t, msg.fl_rpm))
            
        elif topic == "/vehicle/fr_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            t = get_true_time(msg, bag_t_ns)
            data["fr_rpm"].append((t, msg.fr_rpm))
            
        elif topic == "/vehicle/rl_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            t = get_true_time(msg, bag_t_ns)
            data["rl_rpm"].append((t, msg.rl_rpm))
            
        elif topic == "/vehicle/rr_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            t = get_true_time(msg, bag_t_ns)
            data["rr_rpm"].append((t, msg.rr_rpm))
            
        elif topic == "/vehicle/motor_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            t = get_true_time(msg, bag_t_ns)
            data["motor_rpm"].append((t, msg.rr_rpm))
            
    return data

def synchronize_event_based(data):
    print("Synchronizing telemetry based on true event times (Forward Fill)...")
    
    # 1. Convert lists to DataFrames and set the exact nanosecond 'time' as the index
    dfs = []
    if data["control"]: dfs.append(pd.DataFrame(data["control"], columns=["time", "throttle_fl", "throttle_fr", "throttle_rl", "throttle_rr", "steering"]).set_index("time"))
    if data["pose"]: dfs.append(pd.DataFrame(data["pose"], columns=["time", "real_x", "real_y", "real_yaw"]).set_index("time"))
    if data["velocities"]: dfs.append(pd.DataFrame(data["velocities"], columns=["time", "real_vx", "real_vy", "real_yaw_rate"]).set_index("time"))
    if data["fl_rpm"]: dfs.append(pd.DataFrame(data["fl_rpm"], columns=["time", "real_fl_rpm"]).set_index("time"))
    if data["fr_rpm"]: dfs.append(pd.DataFrame(data["fr_rpm"], columns=["time", "real_fr_rpm"]).set_index("time"))
    if data["rl_rpm"]: dfs.append(pd.DataFrame(data["rl_rpm"], columns=["time", "real_rl_rpm"]).set_index("time"))
    if data["rr_rpm"]: dfs.append(pd.DataFrame(data["rr_rpm"], columns=["time", "real_rr_rpm"]).set_index("time"))
    if data["motor_rpm"]: dfs.append(pd.DataFrame(data["motor_rpm"], columns=["time", "real_motor_rpm"]).set_index("time"))
    
    if not dfs:
        raise ValueError("No data found in topics.")
        
    # 2. Combine all dataframes on their time index (Outer Join)
    merged = pd.concat(dfs, axis=1)
    
    # 3. Handle messages that arrive at the exact same nanosecond by taking the latest data
    merged = merged.groupby(merged.index).last()
    
    # 4. Sort chronologically
    merged = merged.sort_index()
    
    # 5. Forward-fill the missing values (Zero-Order Hold behavior)
    merged = merged.ffill()
    
    # 6. Drop the initial rows before all sensors have published at least once
    merged = merged.dropna()
    
    # 7. Convert the index back to a standard 'time' column
    merged = merged.reset_index()
    
    # 8. Calculate custom timing columns
    merged["timestamp_s"] = (merged["time"] - merged["time"].iloc[0]) / 1e9
    
    # Calculate dt based on the timestep of the previous message read
    merged["dt"] = merged["time"].diff() / 1e9
    merged["dt"] = merged["dt"].fillna(0.0)  # The first step has a dt of 0
    
    # 9. Clean up and reorder columns, dropping the raw nanosecond 'time'
    merged = merged.drop(columns=["time"])
    cols = ["timestamp_s", "dt"] + [c for c in merged.columns if c not in ["timestamp_s", "dt"]]
    
    return merged[cols]

def main():
    print("=== True-Time Event Rosbag to CSV Converter ===")
    
    parser = argparse.ArgumentParser(description="Convert MCAP Rosbags to synchronized CSV files.")
    parser.add_argument("bag_path", nargs="?", default=None, help="Path to the ROS2 MCAP bag directory or file")
    parser.add_argument("--output", "-o", dest="output_filename", default=None, help="Output CSV filename (e.g., run.csv)")
    
    args = parser.parse_args()
    
    # 1. Handle Bag Path (CLI Argument or Interactive Prompt)
    bag_path = args.bag_path
    if not bag_path:
        bag_path = input("Enter the path to the ROS2 MCAP bag directory or file:\n> ").strip()
    if not bag_path:
        print("Error: Bag path cannot be empty.")
        sys.exit(1)
        
    # 2. Handle Output Filename (CLI Argument or Interactive Prompt)
    output_filename = args.output_filename
    if not output_filename:
        output_filename = input("Enter the output CSV filename (e.g., acceleration_run.csv):\n> ").strip()
    if not output_filename:
        print("Error: Output filename cannot be empty.")
        sys.exit(1)

    # 3. Automatically route output to the tuning_csvs folder
    output_path = os.path.join("src", "invictasim", "tuning_csvs", output_filename)
    
    try:
        raw_data = read_bag_data(bag_path)
        df_synced = synchronize_event_based(raw_data)
        
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        
        df_synced.to_csv(output_path, index=False)
        print(f"\n✅ Conversion complete! Extracted {len(df_synced)} asynchronous events.")
        print(f"✅ Saved to: {output_path}")
        
    except Exception as e:
        print(f"\n❌ An error occurred during processing: {e}")

if __name__ == "__main__":
    main()