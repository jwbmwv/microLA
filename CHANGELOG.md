# Changelog

All notable changes are summarized here.

## [0.0.3] - 2026-07-25

- **Sensor fusion**: Corrected timestamp, calibration, reference-pose, and measurement-rejection behavior with deterministic trace-replay coverage.
- **Embedded validation**: Added target compile/resource reports, alignment sanitizer coverage, and a Cortex-M4F Renode runtime CI lane.
- **Release engineering**: Added a release contract, explicit validation boundaries, and an ABI-specific estimator-state planning tool.

## [0.0.2] - 2026-05-31

Comprehensive code review and quality improvements:

- **Safety**: NaN propagation for error detection (safe_sqrt, quaternion inverse, division by zero)
- **Performance**: Cached identity matrices, analytical 4×4 inverse (9.22× faster), cache-optimized matrix multiply, AVX-512 support, branch prediction hints with [[likely]]/[[unlikely]]
- **Correctness**: Kalman filter covariance symmetry, Ray-AABB epsilon checks, PlantUML diagram fixes
- **Robustness**: Overflow protection in hypot3, magic constant extraction, timestamp precision warnings
- **Documentation**: Enhanced API docs, sensor fusion failure modes guide, filtering strategies
- **CI/CD**: Automated code snippet verification, Doxygen generation pipeline
- **Testing**: 537 tests passing, updated to reflect NaN propagation behavior

## [0.0.1] - 2026-01-31

Initial release: header-only C++ linear algebra library for embedded and real-time systems.

- Header-only templates: `Vec<T,N>`, `Mat<T,R,C>`, `Quaternion<T>`
- Zero dynamic allocation and POD-compatible types
- SIMD-aware implementations with platform-specific fast paths
- Compile-time rotation helpers and constexpr-friendly utilities
- Safety utilities (safe math, saturating arithmetic)
- Basic benchmarks and tests included

Current Version: **0.0.3**
