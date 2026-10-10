# SIMD Optimization Guide

MicroLA requires C++20. Optional backends select implementations at compile time;
they do not guarantee a speedup or qualify a processor for production use. The
[release contract](RELEASE_CONTRACT.md) defines the supported validation boundary.

## Implemented Paths

- Generic float matrix multiplication streams right-hand rows into an output row.
  The column loop is contiguous and can be vectorized by the compiler without
  gathers or horizontal reductions. Rectangular dimensions and scalar tails are
  covered by regression tests.
- NEON float 4x4 multiplication loads right-hand rows and accumulates each output
  row. AArch64 uses FMA when available; the other branch uses multiply-accumulate.
  There is no separate NEON 3x3 matrix-matrix specialization.
- CMSIS-DSP float multiplication delegates to `arm_mat_mult_f32`. The dependency,
  compiler ABI, dimensions, and target instruction support must match the application.
- Other scalar types retain the generic small-matrix and blocked large-matrix paths.
- Vec, Quaternion, and view operations use selected backend intrinsics or portable
  helpers where implemented. SIMD availability does not imply every method uses it.
- Floating-point Vec/Quaternion normalization first scales by the largest component.
  It uses bounded dot products and square roots rather than relying on an unscaled
  reciprocal-square-root estimate. Zero and non-finite inputs are returned unchanged.
- RVV branches are experimental and require a compatible intrinsic API and vector
  target. Scalar RISC-V compile coverage is not RVV execution coverage.
- MVE, SVE, FP16, and fixed-point kernels are not established by these SIMD claims.

## Configuration

Use the library's CMake options, not `CONFIG_*` CMake cache variables:

```bash
cmake --preset benchmark-ci -DMICROLA_ENABLE_NEON=ON
cmake --build --preset benchmark-ci --target bench_matrix_multiply
```

Only enable NEON on an appropriate ARM toolchain/target. CMSIS also requires its
headers and library in the consumer's build. Define backend preprocessor switches
before including a MicroLA header; see
[compiler_features.hpp](../include/microla/compiler_features.hpp) for detection.

Do not add `-mfpu=neon` to AArch64 compilation. Do not use `-ffast-math` when relying
on finite-value validation or IEEE non-finite behavior.

## Memory And Alignment

Owning types use `MICROLA_DATA_ALIGNMENT`, not a universal 16-byte alignment.
Use `sizeof` and `alignof` for the actual compiler and macro configuration.

| Type | Payload Bytes | Size At Alignment 4 | Size At Alignment 16 |
|------|---------------|---------------------|----------------------|
| Vec<float,2> | 8 | 8 | 16 |
| Vec<float,3> | 12 | 12 | 16 |
| Vec<float,4> | 16 | 16 | 16 |
| Quaternion<float> | 16 | 16 | 16 |
| Mat<float,3,3> | 36 | 36 | 48 |
| Mat<float,4,4> | 64 | 64 | 64 |

Larger configured alignment can add more padding. Alignment alone does not
guarantee cache-line placement, DMA coherency, or permission to read past the
declared data array. Use padding-aware helpers for three-element vectors.

## Reproducible Benchmarks

```bash
cmake --preset benchmark-ci
cmake --build --preset benchmark-ci --target bench_matrix_multiply
./build/benchmark-ci/benchmarks/bench_matrix_multiply \
  --benchmark_filter=bm_matrix_row_streaming \
  --benchmark_repetitions=3 --benchmark_report_aggregates_only=true
```

Each dimension registers a library product (`false`) and a row-streaming reference
(`true`). Inputs are modified and protected from constant folding. Compare within
one executable, with identical compiler flags and machine state.

The 2026-10-09 native host probe used C++20, `-O3 -march=native`, SIMD autodetection,
changing inputs, a checksum, and the best of three rounds:

| Float Size | Previous Gather Path | Row-Streaming Path | Reference |
|------------|----------------------|--------------------|-----------|
| 4x4 | 36.3 ns | 12.2 ns | 11.4 ns |
| 6x6 | 95.3 ns | 14.5 ns | 14.4 ns |
| 32x32 | 9734.0 ns | 1502.7 ns | 1438.3 ns |
| 64x64 | 77429.8 ns | 9221.4 ns | 9534.4 ns |

The updated product matched the reference exactly for that input set. These are
host observations, not ARM/RVV timing guarantees or worst-case execution bounds.
The repository benchmark uses a different harness and need not reproduce these
absolute times. Previous illustrative Cortex-A/Cortex-M speedup tables have been
removed because they did not carry reproducible target evidence.

## Accuracy And Target Validation

FMA and different accumulation orders can change rounding. Compare against a
trusted reference with tolerances suitable for the scalar type, conditioning,
dimension, and application. A fast kernel is not necessarily a more accurate one.

Before deploying an optional backend, compile its actual branches, execute its
regressions on the target, and measure latency, stack, and persistent memory using
the shipping compiler flags. No physical-board timing was performed for this update.

## References

- [NEON Intrinsics](https://developer.arm.com/architectures/instruction-sets/intrinsics/)
- [CMSIS-DSP](https://arm-software.github.io/CMSIS-DSP/latest/)
- [Performance](../PERFORMANCE.md)
- [Release Contract](RELEASE_CONTRACT.md)
