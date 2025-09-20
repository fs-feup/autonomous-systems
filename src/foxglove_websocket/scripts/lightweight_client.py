import asyncio
import websockets
import json
import subprocess
import sys
import signal
import os

# Try to import ROS2 components, handle missing dependencies gracefully
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import Vector3Stamped

    ROS2_AVAILABLE = True

    # Try to import custom interfaces, but don't fail if not available
    try:
        from custom_interfaces.msg import Velocities

        CUSTOM_INTERFACES_AVAILABLE = True
    except ImportError:
        print(
            "Warning: custom_interfaces not available. Some message types may not work."
        )
        CUSTOM_INTERFACES_AVAILABLE = False

except ImportError as e:
    print(f"Error importing ROS2 dependencies: {e}")
    print(
        "Make sure you have sourced the ROS2 setup: source /opt/ros/humble/setup.bash"
    )
    print("And built your workspace: colcon build && source install/setup.bash")
    sys.exit(1)

import threading


class TopicRelayNode(Node):
    def __init__(self, topics_config):
        super().__init__("topic_relay_node")
        self.topics_config = topics_config
        self.subscribers = {}
        self.publishers = {}
        self.setup_topic_relays()

    def setup_topic_relays(self):
        for topic_name, config in self.topics_config.items():
            if isinstance(config, dict) and config.get("enabled", False):
                topic_type = config.get("type", "string")
                self.setup_relay_for_topic(topic_name, topic_type)

    def setup_relay_for_topic(self, topic_name, topic_type):
        if topic_type == "vector3_stamped":
            msg_type = Vector3Stamped
        elif topic_type == "velocities":
            if CUSTOM_INTERFACES_AVAILABLE:
                msg_type = Velocities
            else:
                self.get_logger().warn(
                    f"Custom interfaces not available for velocities type on {topic_name}"
                )
                return
        elif topic_type == "string":
            msg_type = String
        else:
            self.get_logger().warn(f"Unknown topic type: {topic_type} for {topic_name}")
            return

        # Create subscriber to original topic
        subscriber = self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, tn=topic_name: self.relay_callback(msg, tn),
            10,
        )

        # Create publisher for relayed topic (with /relayed prefix)
        relay_topic_name = f"/relayed{topic_name}"
        publisher = self.create_publisher(msg_type, relay_topic_name, 10)

        self.subscribers[topic_name] = subscriber
        self.publishers[topic_name] = publisher

        self.get_logger().info(
            f"Set up relay: {topic_name} -> {relay_topic_name} ({topic_type})"
        )

    def relay_callback(self, msg, topic_name):
        """Relay message from original topic to relayed topic"""
        if topic_name in self.publishers:
            self.publishers[topic_name].publish(msg)


class LightweightClient:
    def __init__(self):
        self.websocket_uri = "ws://localhost:8768"
        self.foxglove_bridge_process = None
        self.topics_config = None
        self.relay_node = None
        self.ros_thread = None

    async def connect_and_receive(self):
        try:
            print(f"Connecting to {self.websocket_uri}...")
            async with websockets.connect(self.websocket_uri) as websocket:
                print("Connected! Waiting for topic configuration...")

                # Receive the JSON config from server
                config_data = await websocket.recv()
                print(f"Received config: {config_data}")

                # Parse the JSON
                self.topics_config = json.loads(config_data)
                print("Parsed topic configuration:")
                for topic_name, config in self.topics_config.items():
                    if isinstance(config, dict) and config.get("enabled", False):
                        print(f"  - {topic_name}: {config.get('type', 'unknown')}")

                # Initialize ROS2 and start topic relay
                self.start_topic_relay()

                # Start Foxglove bridge
                self.start_foxglove_bridge()

                # Keep connection alive
                print(
                    "WebSocket connection established. Topic relay and Foxglove bridge running..."
                )
                print("Relayed topics will have /relayed prefix")
                print("Connect Foxglove Studio to ws://localhost:8765")
                print("Press Ctrl+C to stop...")

                # Keep the connection alive
                try:
                    while True:
                        await asyncio.sleep(1)
                except asyncio.CancelledError:
                    pass

        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cleanup()

    def start_topic_relay(self):
        """Initialize ROS2 and start the topic relay node"""

        def ros_thread_func():
            rclpy.init()
            self.relay_node = TopicRelayNode(self.topics_config)
            try:
                rclpy.spin(self.relay_node)
            except Exception as e:
                print(f"ROS thread error: {e}")
            finally:
                if self.relay_node:
                    self.relay_node.destroy_node()
                rclpy.shutdown()

        self.ros_thread = threading.Thread(target=ros_thread_func, daemon=True)
        self.ros_thread.start()
        print("Started topic relay node")

    def cleanup(self):
        """Clean up all resources"""
        self.stop_foxglove_bridge()
        if self.relay_node:
            self.relay_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def start_foxglove_bridge(self):
        """Start the Foxglove bridge with filtered topics based on config"""
        try:
            print("Starting Foxglove bridge with topic filtering...")

            # Get enabled topics from received config
            enabled_topics = []
            if self.topics_config:
                for topic_name, config in self.topics_config.items():
                    if isinstance(config, dict) and config.get("enabled", False):
                        enabled_topics.append(topic_name)

            if enabled_topics:
                # Create topic filter parameter
                escaped_topics = [topic.replace("/", r"\/") for topic in enabled_topics]
                topic_filter = "|".join([f"^{topic}$" for topic in escaped_topics])
                print(f"Filtering topics: {enabled_topics}")

                # Start foxglove bridge with topic filter
                cmd = [
                    "ros2",
                    "run",
                    "foxglove_bridge",
                    "foxglove_bridge",
                    "--ros-args",
                    "-p",
                    f"topic_whitelist:={topic_filter}",
                ]
            else:
                print("No enabled topics found, starting bridge with all topics")
                cmd = ["ros2", "run", "foxglove_bridge", "foxglove_bridge"]

            self.foxglove_bridge_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # Create new process group
            )
            print("Foxglove bridge started on ws://localhost:8765")

        except Exception as e:
            print(f"Failed to start Foxglove bridge: {e}")

    def stop_foxglove_bridge(self):
        """Stop the Foxglove bridge"""
        if self.foxglove_bridge_process:
            print("Stopping Foxglove bridge...")
            try:
                # Kill the entire process group
                os.killpg(os.getpgid(self.foxglove_bridge_process.pid), signal.SIGTERM)
                self.foxglove_bridge_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                os.killpg(os.getpgid(self.foxglove_bridge_process.pid), signal.SIGKILL)
            except Exception as e:
                print(f"Error stopping Foxglove bridge: {e}")

            self.foxglove_bridge_process = None
            print("Foxglove bridge stopped")

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\nShutting down...")
        self.cleanup()
        sys.exit(0)


async def main():
    client = LightweightClient()

    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, client.signal_handler)
    signal.signal(signal.SIGTERM, client.signal_handler)

    try:
        await client.connect_and_receive()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        client.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
