#!/bin/bash

# Name of the launch command
LAUNCH_COMMAND="ros2 launch launcher vehicle01.launch.py"

# Find the process ID (PID) of the launch command
PID=$(ps aux | grep "$LAUNCH_COMMAND" | grep -v grep | awk '{print $2}')

# Check if the process ID was found and stop it if running
if [ -z "$PID" ]; then
  echo "No running process found for: $LAUNCH_COMMAND. Starting a new instance..."
else
  # Send SIGINT to stop the process (equivalent to Ctrl+C)
  kill -2 "$PID"
  echo "Sent SIGINT to process with PID $PID to stop $LAUNCH_COMMAND"
  # Wait a moment for the process to terminate
  sleep 2
fi

# Relaunch the command
echo "Relaunching: $LAUNCH_COMMAND"
$LAUNCH_COMMAND &
