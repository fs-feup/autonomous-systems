#!/bin/bash
echo "🚀 Starting Cascaded Pure Pursuit Complete Test..."

# Enable controller first
echo "✅ Enabling controller..."
ros2 service call /cascaded_pure_pursuit_test/enable_controller std_srvs/srv/Trigger {}

# Publish Formula Student test track
echo "🏁 Publishing test track..."
ros2 topic pub /test/path nav_msgs/msg/Path '{
    header: {frame_id: "map"},
    poses: [
        {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}},
        {pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}},
        {pose: {position: {x: 20.0, y: 5.0}, orientation: {z: 0.262, w: 0.965}}},
        {pose: {position: {x: 35.0, y: 15.0}, orientation: {w: 1.0}}},
        {pose: {position: {x: 50.0, y: 15.0}, orientation: {w: 1.0}}}
    ]
}' --rate 2 &

# Publish vehicle odometry  
echo "🗺️ Publishing odometry..."
ros2 topic pub /test/odometry nav_msgs/msg/Odometry '{
    header: {frame_id: "odom"},
    child_frame_id: "base_link",
    pose: {pose: {position: {x: 1.0, y: 0.2}, orientation: {z: 0.05, w: 0.999}}},
    twist: {twist: {linear: {x: 12.0}, angular: {z: 0.15}}}
}' --rate 50 &

# Publish IMU
echo "📡 Publishing IMU..."
ros2 topic pub /test/imu sensor_msgs/msg/Imu '{
    header: {frame_id: "imu_link"},
    orientation: {z: 0.05, w: 0.999},
    angular_velocity: {z: 0.15},
    linear_acceleration: {x: 0.5, y: 0.2, z: 9.81}
}' --rate 100 &

# Publish velocity
echo "💨 Publishing velocity..."
ros2 topic pub /test/velocity geometry_msgs/msg/TwistStamped '{
    header: {frame_id: "base_link"},
    twist: {linear: {x: 12.0}, angular: {z: 0.15}}
}' --rate 50 &

echo ""
echo "📊 Monitor these topics in Foxglove:"
echo "   🎯 /cascaded_pure_pursuit_test/debug/outer_loop_command"
echo "   🎮 /cascaded_pure_pursuit_test/debug/inner_loop_command" 
echo "   📈 /cascaded_pure_pursuit_test/debug/path_tracking_metrics"
echo "   🚗 /test/steering_command"
echo ""
echo "✅ Test running! Press Ctrl+C to stop"
wait
