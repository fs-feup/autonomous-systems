cppcheck --enable=all \
    --error-exitcode=1 ./src/ \
    -i test/ -i build/ -i install/ \
    -i can/include/fs-ai_api/ -i can/src/fs-ai_api/ -i ros_can/