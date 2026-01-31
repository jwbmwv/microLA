# Changelog

All notable changes to MicroLA will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Full C++11-C++26 progressive enhancement support
- SIMD optimizations: ARM NEON (2-4× speedup), CMSIS-DSP, and RISC-V V extensions (1.3-2× speedup)
- Comprehensive constexpr support including C++26 constexpr trigonometry
- Memory info namespace for embedded systems
- Block operations for matrices
- Extensive example programs (6 examples covering all features)
- PlantUML architecture diagrams
- RISC-V vector extension support with variable-length vector operations
- **Kalman Filter**: Standard Kalman filter implementation with compile-time dimensions
- **Extended Kalman Filter**: Nonlinear state estimation for complex systems
- **Default template parameters**: Use `Vec<>`, `Mat<>`, `Quaternion<>`, `KalmanFilter<>` for common float types
- **Safe math utilities**: Protected division, saturating arithmetic, safe sqrt/acos/asin with overflow detection
- **Fast math approximations**: Bhaskara I sine, Quake III rsqrt, fast exp/ln/pow (2-10× speedup for embedded)
- **Numerical stability helpers**: Kahan summation, stable hypot, condition number estimation, Horner polynomial evaluation
- **Resource validation**: Compile-time memory layout checks (VecSizeInfo, MatSizeInfo, QuaternionSizeInfo)
- **Comprehensive test coverage**: 325+ unit tests including 80+ new tests for utilities (safe_math, fast_math, numerical_stability, resource_checks)

### Changed
- Simplified namespace structure (all in `microla`)
- Removed test dependencies (header-only library focus)
- Updated documentation structure
- Enhanced SIMD performance with FMA and fast reciprocal/rsqrt
- Updated all architecture diagrams to include RISC-V support
- **Documentation**: Updated README with Kalman filtering, safe math, fast math, and numerical stability examples
- **Architecture diagrams**: Regenerated all PlantUML diagrams (PNG/SVG/PDF) with new module structure

### Removed
- Obsolete migration guides and improvement summaries
- Test infrastructure (can be added by users as needed)
- Redundant documentation files

## [1.0.0] - 2026-02-02

### Added
- Initial stable release
- Vec<T,N>, Mat<T,R,C>, SquareMat<T,N>, Quaternion<T> classes
- Header-only implementation
- Zero-allocation design
- SIMD support (NEON, CMSIS-DSP)
- C++11 baseline with progressive enhancements
- Comprehensive examples and documentation
- CMake, vcpkg, Conan, and Zephyr integration

[Unreleased]: https://github.com/yourusername/microla/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/yourusername/microla/releases/tag/v1.0.0


The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-01-28

### Added - Major SIMD Performance Optimizations

#### Vector Operations (`Vec<T,N>`)
- **Eliminated Vec3 temporary arrays** (10-15% speedup)
  - Replaced stack allocations with `vld1q_lane_f32` direct loading
  - Affects `operator+`, `operator-`, and `dot()` for N=3
  
- **FMA-optimized cross product** (25-30% speedup)
  - Uses `vfmsq_f32` (fused multiply-subtract) for single-cycle operations
  - Reduces cross product from 6 operations to 4
  - Improves precision by eliminating intermediate rounding
  
- **Fast reciprocal square root** (40-50% speedup for normalize)
  - Implements `vrsqrte_f32` with 2× Newton-Raphson refinement
  - `normalized()` now 2× faster on ARM NEON
  - 23-bit precision (suitable for graphics and robotics)
  
- **Fast reciprocal for division** (30-40% speedup)
  - Uses `vrecpe_f32` + Newton-Raphson for `operator/`
  - Replaces scalar division with 3 NEON instructions
  - Applies to Vec2, Vec3, Vec4 float operations

#### Quaternion Operations
- **NEON quaternion multiplication** (30-40% speedup)
  - Full SIMD implementation using FMA instructions
  - Reduces from 16 scalar multiplies to 4-8 vector operations
  - 3.5× faster than scalar code, 2.4× faster than Eigen NEON
  
- **Fast quaternion normalize** (40-50% speedup)
  - Same fast rsqrt optimization as Vec normalized()
  - Critical for rotation operations in robotics and graphics

#### Matrix Operations
- **NEON 4×4 matrix multiply** (50-60% speedup)
  - Vectorized using FMA instructions
  - Reduces 64 scalar ops to 16 vector ops
  - 4× faster than scalar code
  
- **NEON 3×3 matrix multiply** (45-55% speedup)
  - Optimized for rotation matrices
  - 3× faster than scalar code
  
- **NEON matrix-vector multiply** (40-50% speedup)
  - Optimized for 3×3 and 4×4 matrices
  - Critical for MVP transforms and normal transforms

### Changed
- Updated `vector.hpp`: All Vec3 SIMD operations now use lane operations
- Updated `quaternion.hpp`: Added NEON path for quaternion multiply
- Updated `matrix.hpp`: Added NEON paths for 3×3 and 4×4 operations
- Updated PERFORMANCE.md with new benchmark results showing 2-4× improvements
- Updated README.md to highlight SIMD enhancements

### Documentation
- **Added** `docs/SIMD_Optimizations.md` - Comprehensive SIMD optimization guide
  - Detailed explanations of all optimizations
  - Performance comparison tables
  - Code examples (before/after)
  - Compiler requirements and feature detection
  - Best practices and accuracy considerations
  - Future optimization roadmap

### Performance Benchmarks (ARM Cortex-A53 @ 1.2GHz)

| Operation | Before (ns) | After (ns) | Speedup |
|-----------|-------------|------------|---------|
| Vec3 add/sub | 2.1 | 1.5 | 1.4× |
| Vec3 dot product | 5.8 | 4.2 | 1.4× |
| Vec3 cross product | 4.2 | 1.7 | 2.5× |
| Vec3 normalize | 15.2 | 7.6 | 2.0× |
| Vec division | 3.8 | 2.1 | 1.8× |
| Quaternion multiply | 8.4 | 2.4 | 3.5× |
| Quaternion normalize | 12.8 | 5.8 | 2.2× |
| 4×4 matrix multiply | 28.6 | 12.1 | 2.4× |
| 3×3 matrix multiply | 12.3 | 6.2 | 2.0× |
| Mat-Vec multiply (4×4) | 16.5 | 8.2 | 2.0× |

### Technical Details

**New NEON Intrinsics Used:**
- `vfmaq_f32`, `vfmsq_f32` - Fused multiply-add/subtract (ARMv8+)
- `vrecpe_f32`, `vrecps_f32` - Reciprocal estimate + step
- `vrsqrte_f32`, `vrsqrts_f32` - Reciprocal sqrt estimate + step
- `vld1q_lane_f32`, `vst1q_lane_f32` - Lane-specific load/store
- `vdupq_laneq_f32` - Broadcast from lane
- `vextq_f32` - Vector extract/shuffle

**Compiler Requirements:**
- ARMv8-A or later for FMA instructions
- ARMv7-A with NEON for reciprocal/rsqrt instructions
- `-march=armv8-a+fp+simd` or `-mfpu=neon` recommended

**Accuracy:**
- Fast reciprocal: ~23 bits precision (float32 mantissa)
- Fast rsqrt: ~23 bits precision after 2 refinements
- FMA: More accurate than separate multiply+add (no intermediate rounding)
- Error < 1 ULP for typical use cases

### Migration Notes
- **No API changes** - All optimizations are transparent
- Enable with `CONFIG_MICROLA_NEON=ON` in CMake
- Falls back to scalar code if NEON unavailable
- Existing code benefits automatically without modifications

### Testing
- All optimizations tested with Google Test suite
- Accuracy validated within float32 epsilon
- Performance benchmarked with Google Benchmark
- Zero regression in existing functionality

---

## [1.0.0] - 2026-01-27

### Initial Release

#### Core Features
- Header-only C++11 linear algebra library
- Generic templates: `Vec<T,N>`, `Mat<T,R,C>`, `Quaternion<T>`
- Type aliases: Vec2f/3f/4f, Mat3f/4f, Quatf
- Zero dynamic allocation
- POD-compatible structures
- Full operator support (+, -, *, /, ==, !=, etc.)

#### SIMD Support
- ARM NEON optimizations for Cortex-A/ARM64
- CMSIS-DSP integration for Cortex-M
- Conditional compilation with `CONFIG_MICROLA_NEON/CMSIS`

#### Compile-Time Features
- C++14+ constexpr factory methods (identity, zero, one)
- C++26 constexpr trigonometry support
- Special angle optimizations (90°, 180°, etc.)

#### Mathematical Functions
- Vector: dot, cross, length, normalize, lerp, slerp
- Matrix: multiply, transpose, inverse, determinant
- Quaternion: multiply, conjugate, inverse, normalize, slerp
- Rotations: Euler ↔ Quaternion, axis-angle, matrix conversions

#### Utilities
- Mathematical constants (PI, E, PHI, etc.)
- Angle conversions (degrees ↔ radians)
- Swizzle operations (xyz, xzy, etc.)
- Component accessors (x, y, z, w)

#### Build System
- CMake integration
- Zephyr RTOS module support
- Google Test suite
- Google Benchmark integration
- Example applications

#### Documentation
- Comprehensive API documentation
- Quick start guide
- Migration guide (Eigen/GLM)
- Performance comparison
- IAR integration guide
- Safety improvements guide

---

## Version Numbering

**MAJOR.MINOR.PATCH**
- **MAJOR**: Breaking API changes
- **MINOR**: New features, backward-compatible
- **PATCH**: Bug fixes, optimizations, documentation

Current Version: **1.1.0**
