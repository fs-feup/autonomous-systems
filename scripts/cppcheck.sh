cppcheck --enable=all \
    --error-exitcode=1 --inline-suppr ./src/ \
    -i test/ -i build/ -i install/ \
    -I ./src/loc_map/include/ \
    -I ./src/perception/include/ \
    -I ./src/planning/include/ \
    -I ./src/long_control/include/ \
    -I ./src/ros_can/include/ \
    -I ./src/inspection/include/ \
