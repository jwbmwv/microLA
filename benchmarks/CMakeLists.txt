# SPDX-License-Identifier: Apache-2.0
# MicroLA Performance Benchmarks

cmake_minimum_required(VERSION 3.13.1)
project(microla_benchmarks CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MicroLA
if(NOT TARGET microla)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_BINARY_DIR}/microla)
endif()

# Find Google Benchmark
find_package(benchmark QUIET)

if(benchmark_FOUND)
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

    message(STATUS "MicroLA benchmarks enabled")
else()
    message(STATUS "Google Benchmark not found. Benchmarks will not be built.")
    message(STATUS "Install with: sudo apt install libbenchmark-dev (Ubuntu)")
    message(STATUS "           or: brew install google-benchmark (macOS)")
    message(STATUS "           or: vcpkg install benchmark (Windows)")
endif()
