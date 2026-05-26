# MicroLA Performance Benchmarks

This directory contains performance benchmarks for MicroLA using [Google Benchmark](https://github.com/google/benchmark).

## Building Benchmarks

### Prerequisites

Install Google Benchmark:

- CMake 3.23 or later for the shipped benchmark presets

**Ubuntu/Debian:**

```bash
sudo apt install libbenchmark-dev
```

**macOS:**

```bash
brew install google-benchmark
```

**Windows (vcpkg):**

```bash
vcpkg install benchmark
```

**From source:**

```bash
git clone https://github.com/google/benchmark.git
cd benchmark
cmake -E make_directory "build"
cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../
cmake --build "build" --config Release
cmake --build "build" --config Release --target install
```

### Build Commands

```bash
cd microla
cmake --preset benchmark
cmake --build --preset benchmark
```

## Running Benchmarks

```bash
# Run all benchmarks
./build/benchmark-native/benchmarks/bench_matrix_multiply
./build/benchmark-native/benchmarks/bench_simd
./build/benchmark-native/benchmarks/bench_constexpr
./build/benchmark-native/benchmarks/bench_vector_ops
./build/benchmark-native/benchmarks/bench_quaternion
./build/benchmark-native/benchmarks/bench_algorithms
./build/benchmark-native/benchmarks/bench_geometry
./build/benchmark-native/benchmarks/bench_large_matrices
./build/benchmark-native/benchmarks/bench_safe_math
./build/benchmark-native/benchmarks/bench_matrix_param
./build/benchmark-native/benchmarks/bench_kalman
./build/benchmark-native/benchmarks/bench_sensor_fusion
./build/benchmark-native/benchmarks/bench_memory_threaded

# Run with specific filters
./build/benchmark-native/benchmarks/bench_matrix_multiply --benchmark_filter=BM_Matrix4x4.*

# Run with custom time unit
./build/benchmark-native/benchmarks/bench_vector_ops --benchmark_time_unit=us

# Run with JSON output
./build/benchmark-native/benchmarks/bench_simd --benchmark_format=json --benchmark_out=results.json
```

## Benchmark Categories

### 1. Matrix Multiplication (`bench_matrix_multiply.cpp`)

- 3x3 and 4x4 matrix multiplication
- Chain multiplications
- Matrix-vector products
- Transpose operations
- Determinant and inverse calculations

### 2. SIMD Operations (`bench_simd.cpp`)

- Vec3 operations (dot, cross, normalize)
- Vec4 operations (SIMD-optimized)
- Quaternion operations
- Array operations (memory bandwidth)

### 3. Constexpr vs Runtime (`bench_constexpr.cpp`)

- Compile-time vs runtime initialization
- Identity matrix creation
- Zero vector creation
- Special angle rotations
- Lookup table performance

### 4. Vector Operations (`bench_vector_ops.cpp`)

- Length calculations
- Distance computations
- Interpolation (lerp)
- Reflection and projection
- Swizzle operations
- Batch normalization

### 5. Sensor Fusion (`bench_sensor_fusion.cpp`)

- IMU9 Mahony update cost with default runtime calibration
- IMU9 Mahony update cost with non-trivial runtime calibration
- IMU9 MEKF update cost
- Relative pose update and scalar extraction cost

## Interpreting Results

Benchmark output shows:
- **Time**: Average time per operation (ns, μs, ms)
- **CPU**: CPU time (may differ from wall time)
- **Iterations**: Number of times the benchmark ran

Lower times are better. Compare SIMD vs non-SIMD builds to see optimization benefits.

## Optimization Tips

1. **Enable compiler optimizations:**
   ```bash
   cmake --preset benchmark
   cmake --build --preset benchmark
   ```

2. **Enable SIMD:**
   ```bash
   cmake --preset neon   # For ARM NEON builds
   cmake --build --preset neon

   cmake --preset cmsis  # For Cortex-M CMSIS-DSP builds
   cmake --build --preset cmsis
   ```

3. **Reproduce the CI benchmark lane:**
   ```bash
   cmake --preset benchmark-ci
   cmake --build --preset benchmark-ci
   ```

4. **Profile with perf (Linux):**
   ```bash
   perf record --call-graph dwarf ./build/benchmark-native/benchmarks/bench_matrix_multiply
   perf report
   ```

## Expected Performance

On typical hardware (x86_64, ARM Cortex-A):
- 4x4 matrix multiply: ~50-200 ns
- Vec3 dot product: ~5-20 ns
- Vec3 normalize: ~20-100 ns
- Constexpr initialization: ~1-5 ns (compile-time optimized)

SIMD builds should show 2-4x speedup for float operations.
