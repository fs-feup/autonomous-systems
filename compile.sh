
source /opt/ros/humble/setup.bash 2>/dev/null || true
  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
    --parallel-workers 2 \
    --event-handlers console_direct+ \
    --packages-up-to invictasim motion_lib custom_interfaces control \
    --cmake-args -G Ninja -DCMAKE_VERBOSE_MAKEFILE=ON
