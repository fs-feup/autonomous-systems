# Foxglove WebSocket - Lightweight Topic Metadata Server

A lightweight ROS2 WebSocket server that sends topic metadata to clients, which then use the official Foxglove bridge for visualization in Foxglove Studio.

## Architecture

This solution provides a clean separation of concerns:

- **Server**: Ultra-lightweight WebSocket server that sends JSON topic metadata from `node.json`
- **Client**: Python client that receives metadata and launches the official Foxglove bridge
- **Visualization**: Foxglove Studio connects to the official bridge for proper protocol support

## Why This Approach?

Instead of implementing the complex Foxglove WebSocket protocol ourselves, we:
1. Keep the server minimal and fast (only sends JSON metadata)
2. Use the official, battle-tested Foxglove bridge for visualization
3. Avoid schema parsing issues and protocol complexity
4. Maintain easy configurability via JSON

## Files Structure

```
foxglove_websocket/
├── src/
│   └── simple_websocket_server.cpp     # Lightweight C++ WebSocket server
├── include/foxglove_websocket/
│   └── simple_websocket_server.hpp     # Header file
├── scripts/
│   └── lightweight_client.py           # Python client + Foxglove bridge launcher
├── node.json                           # Topic configuration
├── CMakeLists.txt                      # Build configuration
├── package.xml                         # ROS2 package manifest
└── README.md                           # This file
```

## Prerequisites

```bash
# Install dependencies
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge
sudo apt install -y libwebsockets-dev
sudo apt install -y nlohmann-json3-dev
```

## Configuration

Edit `node.json` to configure which topics to expose:

```json
{
  "/imu/angular_velocity": {
    "enabled": true,
    "type": "vector3_stamped"
  },
  "/vehicle_state/velocities": {
    "enabled": true,
    "type": "velocities"
  },
  "/test/string_topic": {
    "enabled": false,
    "type": "string"
  }
}
```

- **enabled**: `true` to include topic, `false` to exclude
- **type**: Topic message type (for metadata only)

## Build

```bash
cd /home/ws
colcon build --packages-select foxglove_websocket
source install/setup.bash
```

## Usage

### Step 1: Start the Lightweight WebSocket Server

```bash
cd /home/ws
source install/setup.bash
ros2 run foxglove_websocket simple_websocket_server
```

The server will:
- Load configuration from `node.json`
- Start WebSocket server on port **8767**
- Send JSON metadata to connecting clients

**Output:**
```
[INFO] [simple_websocket_server]: Starting simple WebSocket server...
[INFO] [simple_websocket_server]: Loaded config: {"/imu/angular_velocity":{"enabled":true,"type":"vector3_stamped"},...}
[INFO] [simple_websocket_server]: WebSocket server started on port 8767
```

### Step 2: Start the Client (launches Foxglove bridge)

```bash
cd /home/ws
python3 src/foxglove_websocket/scripts/lightweight_client.py
```

The client will:
- Connect to WebSocket server (port 8767)
- Receive JSON topic configuration
- Automatically launch Foxglove bridge on port **8765**
- Keep both connections alive

**Output:**
```
Connecting to ws://localhost:8767...
Connected! Waiting for topic configuration...
Received config: {"/imu/angular_velocity":{"enabled":true,"type":"vector3_stamped"},...}
Parsed topic configuration:
  - /imu/angular_velocity: vector3_stamped
  - /vehicle_state/velocities: velocities
Starting Foxglove bridge...
Foxglove bridge started on ws://localhost:8765
WebSocket connection established. Foxglove bridge running...
Connect Foxglove Studio to ws://localhost:8765
Press Ctrl+C to stop...
```

### Step 3: Connect Foxglove Studio

1. Open **Foxglove Studio** (desktop app or web)
2. Choose **"Open Connection"**
3. Select **"Foxglove WebSocket"**
4. Enter URL: `ws://localhost:8765`
5. Click **"Open"**

You should now see all your ROS2 topics available for visualization!

## Architecture Flow

```
┌─────────────────┐    JSON metadata    ┌──────────────────┐
│   WebSocket     │ ←─────────────────→ │  Python Client   │
│   Server        │     (port 8767)     │                  │
│ (C++, port 8767)│                     │                  │
└─────────────────┘                     └──────────────────┘
                                                   │
                                                   │ launches
                                                   ▼
┌─────────────────┐    Foxglove Protocol ┌──────────────────┐
│ Foxglove Studio │ ←─────────────────→  │ Foxglove Bridge  │
│                 │     (port 8765)      │   (Official)     │
└─────────────────┘                      └──────────────────┘
                                                   │
                                                   │ subscribes to
                                                   ▼
                                          ┌──────────────────┐
                                          │   ROS2 Topics    │
                                          │                  │
                                          └──────────────────┘
```

## Stopping

Press **Ctrl+C** in the client terminal. It will:
- Gracefully stop the Foxglove bridge
- Close WebSocket connections
- Clean up all processes

## Troubleshooting

### Port Already in Use
If port 8767 is busy, edit `simple_websocket_server.cpp`:
```cpp
int port_ = 8768; // Change to different port
```

### Foxglove Bridge Not Found
```bash
sudo apt install -y ros-humble-foxglove-bridge
```

### WebSocket Connection Failed
- Ensure server is running first
- Check firewall settings
- Verify port numbers match

### No Topics in Foxglove Studio
- Ensure ROS2 topics are publishing data
- Check topic names in `node.json` match actual topics
- Verify Foxglove bridge is connected to ROS2

## Performance Benefits

- **Minimal CPU usage**: Server only sends JSON, no message processing
- **Fast startup**: No complex protocol initialization
- **Low memory**: Simple WebSocket + JSON parsing only
- **Reliable**: Uses official Foxglove components for visualization
- **Maintainable**: Clean, simple codebase

## Development

To modify topic configuration:
1. Edit `node.json`
2. Restart the server
3. Restart the client
4. Reconnect Foxglove Studio

The server will automatically load the new configuration and send it to clients.