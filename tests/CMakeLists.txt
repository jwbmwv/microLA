# SPDX-License-Identifier: MIT
# Test suite for microla

cmake_minimum_required(VERSION 3.14)

# Find Google Test
find_package(GTest REQUIRED)
include(GoogleTest)

# Include microla headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# Collect all test source files
file(GLOB_RECURSE TEST_SOURCES
    "google/test_constants.cpp"
    "google/test_vector.cpp"
    "google/test_matrix.cpp"
    "google/test_quaternion.cpp"
    "google/test_geometry.cpp"
    "google/test_integration.cpp"
    "google/test_kalman.cpp"
    "google/test_safe_math.cpp"
    "google/test_fast_math.cpp"
    "google/test_numerical.cpp"
    "google/test_resource.cpp"
)

# Create test executable
add_executable(microla_gtests ${TEST_SOURCES})

# Link against GTest only
target_link_libraries(microla_gtests
    PRIVATE
    GTest::gtest
    GTest::gtest_main
)

# Set C++ standard
target_compile_features(microla_gtests PRIVATE cxx_std_11)

# Discover tests
gtest_discover_tests(microla_gtests)

# Add custom test target
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS microla_gtests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
