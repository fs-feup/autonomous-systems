#pragma once

#include <vector>
#include <cstdint>
using namespace std;



struct PhysicsState {
    // Position in Unreal units (cm)
    float x, y, z;

    // Orientation (Unreal uses Quaternions)
    float qx, qy, qz, qw;

    // Linear and angular velocities
    float vx, vy, vz;
    float roll_rate, pitch_rate, yaw_rate;

    // Optional: sensor data placeholders
    vector<float> lidar;
    vector<uint8_t> camera_image;


    PhysicsState(int num_lidar_points , unsigned long camera_buffer_size)
        : lidar(num_lidar_points, 0.0f), camera_image(camera_buffer_size, 0) {} // This constructor allows the vectors to be initialized using the config file
};

