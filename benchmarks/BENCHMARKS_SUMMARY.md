# Benchmarks Summary

## Overview

The benchmark suite currently builds 13 executables that cover low-level SIMD kernels, fixed-size matrix and vector operations, quaternion math, Kalman filtering, sensor-fusion update paths, and memory/threading behavior.

## Benchmark Targets

- `bench_matrix_multiply.cpp`: Small dense matrix products and related matrix-vector workloads.
- `bench_simd.cpp`: SIMD helper throughput and vectorized kernel comparisons.
- `bench_constexpr.cpp`: Compile-time versus runtime construction and lookup costs.
- `bench_vector_ops.cpp`: Vec arithmetic, normalization, interpolation, and batch-style vector workloads.
- `bench_quaternion.cpp`: Quaternion composition, normalization, interpolation, and rotation costs.
- `bench_algorithms.cpp`: Core numerical routines such as decomposition and solver-oriented workflows.
- `bench_geometry.cpp`: Geometry intersection and containment checks.
- `bench_large_matrices.cpp`: Larger fixed-size matrix workloads and cache-sensitive access patterns.
- `bench_safe_math.cpp`: Safe-math utility overhead and edge-case handling.
- `bench_matrix_param.cpp`: Parameterized matrix scenarios across representative sizes.
- `bench_kalman.cpp`: Small Kalman filter predict/update pipeline cost.
- `bench_sensor_fusion.cpp`: IMU9 Mahony and MEKF updates, runtime calibration overhead, and relative-angle estimation.
- `bench_memory_threaded.cpp`: Memory-access and threaded benchmark scenarios.

## Build Configuration

- Benchmark targets are declared in `benchmarks/CMakeLists.txt`.
- The standard CI-style benchmark build uses the `benchmark-ci` CMake preset.
- The local host-tuned benchmark build uses the `benchmark` CMake preset.

## Running the Benchmarks

### Run all benchmarks

```bash
cd build/benchmarks
./bench_matrix_multiply
./bench_simd
./bench_constexpr
./bench_vector_ops
./bench_quaternion
./bench_algorithms
./bench_geometry
./bench_large_matrices
./bench_safe_math
./bench_matrix_param
./bench_kalman
./bench_sensor_fusion
./bench_memory_threaded
```

### Run a targeted sensor-fusion smoke check

```bash
./bench_sensor_fusion --benchmark_filter='bm_sensor_fusion_mahony_identity_update|bm_sensor_fusion_ekf_update'
```

### Generate JSON output

```bash
./bench_sensor_fusion --benchmark_format=json > sensor_fusion_results.json
```

## Coverage Summary

The suite covers scalar and SIMD math kernels, transformation-heavy math types, representative estimation pipelines, and higher-level application-style workloads that are sensitive to both arithmetic and memory behavior.
