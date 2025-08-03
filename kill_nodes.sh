# Keywords to match against running ROS 2 processes
NODE_PATTERNS=(
  "slam"
  "planning_adapter"
  "control_adapter"
  "perception_adapter"
  "velocity_estimation"
  "inspection"
)


echo "Stopping nodes by matching process patterns..."
for pattern in "${NODE_PATTERNS[@]}"; do
    echo "Searching for processes matching: $pattern"
    # Find matching processes (exclude grep itself)
    pids=$(ps -eo pid,cmd | grep "$pattern" | grep -v grep | awk '{print $1}')
    for pid in $pids; do
        echo "Sending SIGINT to PID $pid (matched pattern '$pattern')"
        kill -2 "$pid"
    done
    if [ -z "$pids" ]; then
        echo "No processes found for pattern: $pattern"
    fi
done
