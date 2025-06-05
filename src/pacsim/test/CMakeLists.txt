enable_testing()
# Using googletest for unit tests
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/release-1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

add_executable(unit_tests
  pacsim_test.cpp
  ../src/pacsim_main.cpp
)

target_include_directories(unit_tests PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(unit_tests 
gtest_main)

include(GoogleTest)
gtest_discover_tests(unit_tests)