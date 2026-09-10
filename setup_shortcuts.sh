#!/usr/bin/env bash

set -euo pipefail

BASHRC="$HOME/.bashrc"

# By default, use the directory where this script is executed from.
# You can also pass the workspace path explicitly:
# ./setup_shortcuts.sh /home/diogo/ws
ROS_WS="${1:-$(pwd)}"
ROS_WS="$(cd "$ROS_WS" && pwd)"

START_MARKER="# >>> ROS2 WORKSPACE SHORTCUTS >>>"
END_MARKER="# <<< ROS2 WORKSPACE SHORTCUTS <<<"

touch "$BASHRC"

# Backup current bashrc
cp "$BASHRC" "$BASHRC.backup.$(date +%Y%m%d_%H%M%S)"

# Remove old shortcuts block if it already exists
sed -i "/$START_MARKER/,/$END_MARKER/d" "$BASHRC"

cat >> "$BASHRC" <<EOF

$START_MARKER

export ROS_WS="$ROS_WS"

EOF

cat >> "$BASHRC" <<'EOF'
# Automatically source this workspace when opening a new shell
if [ -f "$ROS_WS/install/setup.bash" ]; then
    source "$ROS_WS/install/setup.bash"
fi

# Source workspace manually
_ros_source_ws() {
    if [ -f "$ROS_WS/install/setup.bash" ]; then
        source "$ROS_WS/install/setup.bash"
    else
        echo "Could not find: $ROS_WS/install/setup.bash"
        return 1
    fi
}

alias s='_ros_source_ws'

# Compile package and its dependencies
# Example:
# c planning
# c planning perception control
c() {
    if [ "$#" -lt 1 ]; then
        echo "Usage: c <package_name> [more_packages...]"
        echo "Example: c planning"
        return 1
    fi

    local old_dir="$PWD"

    cd "$ROS_WS" || return 1

    MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
        --parallel-workers 1 --executor sequential \
        --packages-up-to "$@" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    local build_status=$?

    cd "$old_dir" || return "$build_status"

    if [ "$build_status" -eq 0 ]; then
        _ros_source_ws
    fi

    return "$build_status"
}

# Compile only the selected package
# Example:
# cs planning
# cs planning perception control
cs() {
    if [ "$#" -lt 1 ]; then
        echo "Usage: cs <package_name> [more_packages...]"
        echo "Example: cs planning"
        return 1
    fi

    local old_dir="$PWD"

    cd "$ROS_WS" || return 1

    MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
        --parallel-workers 1 --executor sequential \
        --packages-select "$@" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    local build_status=$?

    cd "$old_dir" || return "$build_status"

    if [ "$build_status" -eq 0 ]; then
        _ros_source_ws
    fi

    return "$build_status"
}

# Delete build/install folders for one or more specific packages
# Example:
# del control
# del planning perception control
del() {
    if [ "$#" -lt 1 ]; then
        echo "Usage: del <package_name> [more_packages...]"
        echo "Example: del control"
        return 1
    fi

    local old_dir="$PWD"

    cd "$ROS_WS" || return 1

    for pkg in "$@"; do
        # Safety check to avoid dangerous paths
        case "$pkg" in
            ""|/*|*/*|*..*)
                echo "Invalid package name: $pkg"
                cd "$old_dir" || return 1
                return 1
                ;;
        esac

        echo "Deleting:"
        echo "  build/$pkg"
        echo "  install/$pkg"

        rm -rf -- "build/$pkg" "install/$pkg"
    done

    cd "$old_dir" || return 1
}

# Go to workspace root
alias ws='cd "$ROS_WS"'

# Launch shortcuts
alias fox='_ros_source_ws && ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
alias sim='_ros_source_ws && ros2 launch invictasim invictasim.launch.py'
alias pacsim='_ros_source_ws && ros2 launch pacsim autocross.launch.py'

alias perc='_ros_source_ws && ros2 launch perception perception.launch.py'
alias ve='_ros_source_ws && ros2 launch velocity_estimation velocity_estimation.launch.py'
alias slam='_ros_source_ws && ros2 launch slam slam.launch.py'
alias plan='_ros_source_ws && ros2 launch planning planning.launch.py'
alias control='_ros_source_ws && ros2 launch control control.launch.py'

# Common ROS shortcuts
alias nodes='_ros_source_ws && ros2 node list'
alias topics='_ros_source_ws && ros2 topic list'
alias services='_ros_source_ws && ros2 service list'
alias params='_ros_source_ws && ros2 param list'

EOF

cat >> "$BASHRC" <<EOF

$END_MARKER
EOF

echo "Shortcuts installed in $BASHRC"
echo "Workspace path: $ROS_WS"
echo ""
echo "Run this now:"
echo "source ~/.bashrc"