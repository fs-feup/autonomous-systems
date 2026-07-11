#!/usr/bin/env python3
import os
import sys
import argparse
import pandas as pd
import numpy as np

# ROS 2 deserialization
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Custom and standard messages
from custom_interfaces.msg import ControlCommand, Pose, Velocities, WheelRPM

def read_bag_data(bag_path):
    print(f"Reading ROS2 bag: {bag_path}")
    
    # Setup bag reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    
    data = {
        "control": [],       # (timestamp_ns, throttle_fl, throttle_fr, throttle_rl, throttle_rr, steering)
        "pose": [],          # (timestamp_ns, x, y, yaw)
        "velocities": [],    # (timestamp_ns, vx, vy, yaw_rate)
        "fl_rpm": [],        # (timestamp_ns, rpm)
        "fr_rpm": [],        # (timestamp_ns, rpm)
        "rl_rpm": [],        # (timestamp_ns, rpm)
        "rr_rpm": [],        # (timestamp_ns, rpm)
        "motor_rpm": []      # (timestamp_ns, rpm)
    }
    
    while reader.has_next():
        topic, data_serialized, timestamp_ns = reader.read_next()
        if topic not in topic_types:
            continue
            
        msg_type_str = topic_types[topic]
        
        if topic == "/control/command":
            msg = deserialize_message(data_serialized, ControlCommand)
            data["control"].append((
                timestamp_ns,
                msg.throttle.front_left,
                msg.throttle.front_right,
                msg.throttle.rear_left,
                msg.throttle.rear_right,
                msg.steering
            ))
        elif topic == "/state_estimation/vehicle_pose":
            msg = deserialize_message(data_serialized, Pose)
            data["pose"].append((timestamp_ns, msg.x, msg.y, msg.theta))
        elif topic == "/state_estimation/velocities":
            msg = deserialize_message(data_serialized, Velocities)
            data["velocities"].append((timestamp_ns, msg.velocity_x, msg.velocity_y, msg.angular_velocity))
        elif topic == "/vehicle/fl_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            data["fl_rpm"].append((timestamp_ns, msg.fl_rpm))
        elif topic == "/vehicle/fr_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            data["fr_rpm"].append((timestamp_ns, msg.fr_rpm))
        elif topic == "/vehicle/rl_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            data["rl_rpm"].append((timestamp_ns, msg.rl_rpm))
        elif topic == "/vehicle/rr_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            data["rr_rpm"].append((timestamp_ns, msg.rr_rpm))
        elif topic == "/vehicle/motor_rpm":
            msg = deserialize_message(data_serialized, WheelRPM)
            # Motor RPM is in rr_rpm field of standard WheelRPM message
            data["motor_rpm"].append((timestamp_ns, msg.rr_rpm))
            
    return data

def resample_and_synchronize(data, frequency_hz=200.0):
    print("Resampling and synchronizing telemetry data at 200 Hz...")
    
    # Convert lists to Pandas DataFrames
    df_ctrl = pd.DataFrame(data["control"], columns=["time", "throttle_fl", "throttle_fr", "throttle_rl", "throttle_rr", "steering"])
    df_pose = pd.DataFrame(data["pose"], columns=["time", "real_x", "real_y", "real_yaw"])
    df_vel = pd.DataFrame(data["velocities"], columns=["time", "real_vx", "real_vy", "real_yaw_rate"])
    df_fl = pd.DataFrame(data["fl_rpm"], columns=["time", "real_fl_rpm"])
    df_fr = pd.DataFrame(data["fr_rpm"], columns=["time", "real_fr_rpm"])
    df_rl = pd.DataFrame(data["rl_rpm"], columns=["time", "real_rl_rpm"])
    df_rr = pd.DataFrame(data["rr_rpm"], columns=["time", "real_rr_rpm"])
    df_mot = pd.DataFrame(data["motor_rpm"], columns=["time", "real_motor_rpm"])
    
    # Find minimum and maximum times across all received topics
    all_dfs = [df_ctrl, df_pose, df_vel, df_fl, df_fr, df_rl, df_rr, df_mot]
    valid_dfs = [df for df in all_dfs if not df.empty]
    
    if not valid_dfs:
        raise ValueError("No data found in topics.")
        
    t_start = max(df["time"].iloc[0] for df in valid_dfs)
    t_end = min(df["time"].iloc[-1] for df in valid_dfs)
    
    # 200 Hz target timestamps
    dt_ns = int(1e9 / frequency_hz)
    target_times = np.arange(t_start, t_end, dt_ns)
    
    synced = pd.DataFrame({"time": target_times})
    synced["timestamp_s"] = (synced["time"] - t_start) / 1e9
    
    # Interpolation function for continuous variables
    def interpolate_field(df, field_name):
        if df.empty:
            synced[field_name] = 0.0
            return
        # Sort and remove duplicate timestamps
        df_sorted = df.drop_duplicates(subset=["time"]).sort_values("time")
        synced[field_name] = np.interp(synced["time"], df_sorted["time"], df_sorted[field_name])
        
    # Zero-order hold (last seen value) for control inputs to prevent predicting commands before they occur
    def zoh_field(df, field_name):
        if df.empty:
            synced[field_name] = 0.0
            return
        df_sorted = df.drop_duplicates(subset=["time"]).sort_values("time")
        # Find index of last element <= target time
        indices = np.searchsorted(df_sorted["time"], synced["time"], side="right") - 1
        indices = np.clip(indices, 0, len(df_sorted) - 1)
        synced[field_name] = df_sorted[field_name].iloc[indices].values
        
    # Apply interpolation
    interpolate_field(df_pose, "real_x")
    interpolate_field(df_pose, "real_y")
    interpolate_field(df_pose, "real_yaw")
    interpolate_field(df_vel, "real_vx")
    interpolate_field(df_vel, "real_vy")
    interpolate_field(df_vel, "real_yaw_rate")
    interpolate_field(df_fl, "real_fl_rpm")
    interpolate_field(df_fr, "real_fr_rpm")
    interpolate_field(df_rl, "real_rl_rpm")
    interpolate_field(df_rr, "real_rr_rpm")
    interpolate_field(df_mot, "real_motor_rpm")
    
    # Apply ZOH to control commands
    zoh_field(df_ctrl, "throttle_fl")
    zoh_field(df_ctrl, "throttle_fr")
    zoh_field(df_ctrl, "throttle_rl")
    zoh_field(df_ctrl, "throttle_rr")
    zoh_field(df_ctrl, "steering")
    
    return synced.drop(columns=["time"])

def main():
    parser = argparse.ArgumentParser(description="Convert MCAP Rosbags to synchronized CSV files for C++ Offline Vehicle Model Optimizer")
    parser.add_argument("bag_path", help="Path to the ROS2 MCAP bag directory or file")
    parser.add_argument("--output", "-o", required=True, help="Path to write the resampled output CSV file")
    args = parser.parse_args()
    
    raw_data = read_bag_data(args.bag_path)
    df_synced = resample_and_synchronize(raw_data)
    
    # Ensure parent directories exist
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    df_synced.to_csv(args.output, index=False)
    print(f"Successfully wrote synchronized telemetry to {args.output}")

if __name__ == "__main__":
    main()
