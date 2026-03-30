# SPDX-License-Identifier: Apache-2.0
# Test suite for microla

cmake_minimum_required(VERSION 3.14)

# Always fetch GoogleTest with FetchContent to ensure availability on CI
include(FetchContent)

# When building with MSVC ensure googletest is built with the shared CRT
# to match projects using /MD. This prevents linker errors from mixing
# static and dynamic CRT runtimes (LIBCMT vs MSVCP/ MSVCRT).
if(MSVC)
    set(gtest_force_shared_crt ON CACHE BOOL "Force googletest to use shared CRT (/MD)" FORCE)
    if(NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" CACHE STRING "Default MSVC runtime library for targets built in this directory" FORCE)
    endif()
endif()

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
)
FetchContent_MakeAvailable(googletest)
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
    "google/test_matrix_view.cpp"
    "google/test_vector_view.cpp"
    "google/test_compiler_features.cpp"
    "google/test_simd_helpers.cpp"
    "google/test_vec_quat_factories.cpp"
    "google/test_traits_and_constexpr.cpp"
    "google/test_coverage_gaps.cpp"
    "google/test_sensor_fusion.cpp"
)

set(EMBEDDED_CONTRACT_TEST_SOURCES
    "google/test_embedded_contract.cpp"
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
target_compile_features(microla_gtests PRIVATE cxx_std_20)

# When the project-level benchmark preset enables -ffast-math, standard
# library predicates like std::isfinite may be incorrectly optimized to
# assumptions that break test assertions. Forcefully disable fast-math for
# the test target so tests observe standard IEEE semantics.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(microla_gtests PRIVATE -fno-fast-math)
endif()

# Discover tests
gtest_discover_tests(microla_gtests)

add_executable(microla_embedded_contract_gtests ${EMBEDDED_CONTRACT_TEST_SOURCES})

target_link_libraries(microla_embedded_contract_gtests
    PRIVATE
    GTest::gtest
    GTest::gtest_main
)

target_compile_features(microla_embedded_contract_gtests PRIVATE cxx_std_20)
target_compile_definitions(microla_embedded_contract_gtests PRIVATE MICROLA_EMBEDDED)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(microla_embedded_contract_gtests PRIVATE -fno-fast-math)
endif()

gtest_discover_tests(microla_embedded_contract_gtests)

# Add custom test target
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS microla_gtests microla_embedded_contract_gtests
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
