
# Unix Makefiles, not Ninja. Ninja ignores MAKEFLAGS and defaults to one job per core, which
# is what drove the machine out of memory. Make honours -j2, and one colcon worker at a time
# keeps the total to two compiler processes.
source /opt/ros/humble/setup.bash 2>/dev/null || true
  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
    --parallel-workers 1 \
    --executor sequential \
    --event-handlers console_direct+ \
    --packages-up-to invictasim motion_lib custom_interfaces control \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
