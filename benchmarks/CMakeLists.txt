# SPDX-License-Identifier: Apache-2.0
# MicroLA Performance Benchmarks

cmake_minimum_required(VERSION 3.13.1)
project(microla_benchmarks CXX)

# Find MicroLA
if(NOT TARGET microla)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/microla)
endif()

# Build an immutable, Release-configured benchmark dependency instead of using
# whatever binary package happens to be installed on the runner.
include(FetchContent)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    google_benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG 344117638c8ff7e239044fd0fa7085839fc03021
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(google_benchmark)

# Matrix multiplication benchmark
    add_executable(bench_matrix_multiply bench_matrix_multiply.cpp)
    target_link_libraries(bench_matrix_multiply
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # SIMD vs non-SIMD benchmark
    add_executable(bench_simd bench_simd.cpp)
    target_link_libraries(bench_simd
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Constexpr vs runtime benchmark
    add_executable(bench_constexpr bench_constexpr.cpp)
    target_link_libraries(bench_constexpr
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Vector operations benchmark
    add_executable(bench_vector_ops bench_vector_ops.cpp)
    target_link_libraries(bench_vector_ops
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Quaternion operations benchmark
    add_executable(bench_quaternion bench_quaternion.cpp)
    target_link_libraries(bench_quaternion
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Numerical algorithms benchmark
    add_executable(bench_algorithms bench_algorithms.cpp)
    target_link_libraries(bench_algorithms
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Geometry intersections benchmark
    add_executable(bench_geometry bench_geometry.cpp)
    target_link_libraries(bench_geometry
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Large matrices and cache efficiency benchmark
    add_executable(bench_large_matrices bench_large_matrices.cpp)
    target_link_libraries(bench_large_matrices
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    # Additional microbenchmarks
    add_executable(bench_safe_math bench_safe_math.cpp)
    target_link_libraries(bench_safe_math
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    add_executable(bench_matrix_param bench_matrix_param.cpp)
    target_link_libraries(bench_matrix_param
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    add_executable(bench_kalman bench_kalman.cpp)
    target_link_libraries(bench_kalman
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    add_executable(bench_sensor_fusion bench_sensor_fusion.cpp)
    target_link_libraries(bench_sensor_fusion
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

    add_executable(bench_memory_threaded bench_memory_threaded.cpp)
    target_link_libraries(bench_memory_threaded
        PRIVATE
            microla
            benchmark::benchmark
            benchmark::benchmark_main
    )

message(STATUS "MicroLA benchmarks enabled with Google Benchmark 1.8.3")
