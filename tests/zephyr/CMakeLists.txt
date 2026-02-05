# SPDX-License-Identifier: MIT
# CMakeLists.txt for MicroLA Zephyr Tests

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package (must be called before project())
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(microla_tests)

# Add MicroLA include directory
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

# Add test source files
target_sources(app PRIVATE
    src/main.c
    src/test_vector.cpp
    src/test_matrix.cpp
    src/test_quaternion.cpp
    src/test_integration.cpp
)

# Enable C++17 for MicroLA
set_property(TARGET app PROPERTY CXX_STANDARD 17)
set_property(TARGET app PROPERTY CXX_STANDARD_REQUIRED ON)
