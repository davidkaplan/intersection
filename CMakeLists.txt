cmake_minimum_required(VERSION 3.10)

# Project name
project(intersection)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# May need to add these search paths
#list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
#list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/include/eigen3")

# Find packages
find_package(Eigen3 REQUIRED VERBOSE)
find_package(GTest REQUIRED)

# Add library
add_library(intersection intersection.cpp)
target_include_directories(intersection PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(intersection PUBLIC
    Eigen3::Eigen
)

# Add tests
add_executable(intersection_test test_intersection.cpp)
target_include_directories(intersection_test PRIVATE
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(intersection_test PRIVATE
    intersection
    GTest::GTest
    GTest::Main
)
enable_testing()
include(GoogleTest)
gtest_discover_tests(intersection_test)

# Add executable (main is provided for convenience)
add_executable(main main.cpp)
target_link_libraries(main PRIVATE intersection)
target_compile_options(main PRIVATE -Wall -g)

# Add a custom run target for main
add_custom_target(run
    COMMAND main
    DEPENDS main
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
