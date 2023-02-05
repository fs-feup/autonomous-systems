# CMake generated Testfile for 
# Source directory: /home/marhcouto/Documents/training/experiment1
# Build directory: /home/marhcouto/Documents/training/build/experiment1
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(experiment1_test "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/marhcouto/Documents/training/build/experiment1/test_results/experiment1/experiment1_test.gtest.xml" "--package-name" "experiment1" "--output-file" "/home/marhcouto/Documents/training/build/experiment1/ament_cmake_gtest/experiment1_test.txt" "--command" "/home/marhcouto/Documents/training/build/experiment1/experiment1_test" "--gtest_output=xml:/home/marhcouto/Documents/training/build/experiment1/test_results/experiment1/experiment1_test.gtest.xml")
set_tests_properties(experiment1_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/marhcouto/Documents/training/build/experiment1/experiment1_test" TIMEOUT "60" WORKING_DIRECTORY "/home/marhcouto/Documents/training/build/experiment1" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/galactic/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/marhcouto/Documents/training/experiment1/CMakeLists.txt;39;ament_add_gtest;/home/marhcouto/Documents/training/experiment1/CMakeLists.txt;0;")
subdirs("gtest")
