# MicroLA

A lightweight, header-only C++20 linear algebra library optimized for embedded systems and real-time applications.

## Features

- **Header-only**: No compilation required, just include the headers
- **Fixed-size core types**: `Vec`, `Mat`, and `Quaternion` store data inline with no heap allocation for core arithmetic
- **C-friendly layouts**: Core math types are standard-layout and trivially copyable for DMA and C interop
- **Default template parameters**: Use `Vec<>`, `Mat<>`, `Quaternion<>` for common float types
- **SIMD-aware**: Scalar paths are always available; optional NEON, CMSIS-DSP, and MVE configuration depends on the target, toolchain, and supplied external libraries
  - **NEON**: Intended for NEON-capable Arm application processors
  - **CMSIS-DSP**: Requires an application-provided CMSIS-DSP dependency on supported Cortex-M devices
  - **MVE**: Configuration support is available for Helium-capable Cortex-M profiles; application workloads must measure the resulting code on target
- **C++20 baseline with forward-looking feature gates**: Uses C++20 directly and enables C++23/C++26 improvements when available
- **Embedded-aware APIs**: Caller-provided buffer overloads are available for allocation-sensitive paths such as QR eigenvalue extraction
- **Namespaced**: All classes in `microla` namespace to avoid pollution
- **Versioned**: Runtime version API for compatibility checking
- **Type-safe**: Compile-time dimension checking prevents errors
- **Const-correct**: Full const correctness throughout the API
- **Sanitizer-tested**: CI runs ASan and UBSan configurations on the host test suite
- **Embedded-friendly**: Designed for microcontrollers with limited resources
- **Rich utilities**: Mathematical constants, angle utilities, interpolation, swizzling
- **Sensor fusion ready**: Coordinate transforms, safe normalization, Euler/quaternion conversions, and documented SI-unit conventions
- **Kalman filtering**: Standard and Extended Kalman Filters for state estimation
- **Safe math**: Protected operations with overflow/underflow detection and clamping
- **Fast math**: Optional approximations for applications that choose their accuracy/performance trade-off
- **Numerical stability**: Kahan summation, stable hypot, condition number checks

## Quick Start

### As a Git Submodule

```bash
# Add to your project
git submodule add https://github.com/jwbmwv/microla.git external/microla
git submodule update --init --recursive

# Update CMakeLists.txt
add_subdirectory(external/microla)
target_link_libraries(your_target PUBLIC microla)
```

### As a Zephyr Module

1. Add to your `west.yml`:

```yaml
manifest:
  projects:
    - name: microla
      url: https://github.com/jwbmwv/microla.git
      revision: main
      path: modules/lib/microla
```

2. In your application's `CMakeLists.txt`:

```cmake
# MicroLA is automatically available as a Zephyr module
target_link_libraries(app PUBLIC microla)
```

### Direct Include

For simple projects, just copy `include/microla/` to your include path:

```cpp
// Option 1: Include everything (convenience)
#include <microla/microla.hpp>   // Includes all vector/matrix components
#include <microla/quaternion.hpp>
#include <microla/version.hpp>

// Option 2: Modular includes (faster compile times)
#include <microla/constants.hpp>     // Mathematical constants (can be used independently)
#include <microla/vector.hpp>        // Base Vec<T,N> template
#include <microla/matrix.hpp>        // Base Mat<T,R,C> and SquareMat<T,N> templates
#include <microla/quaternion.hpp>    // Quaternion rotations
#include <microla/safe_math.hpp>     // Safety-critical operations
#include <microla/fast_math.hpp>     // Fast approximations
#include <microla/numerical_stability.hpp>  // Kahan summation, stable hypot
#include <microla/geometry.hpp>      // Ray, Plane, AABB, Sphere, Triangle, Frustum
#include <microla/kalman.hpp>        // Standard Kalman filter
#include <microla/extended_kalman.hpp> // Extended Kalman filter
#include <microla/version.hpp>       // Runtime version API

using namespace microla;

Vec<float, 3> v(1.0f, 2.0f, 3.0f);
Quaternion<float> q(Vec<float, 3>(0, 0, 1), 1.57f);

// Check library version
if (version_at_least(0, 0, 1)) {
  // Use library features
}
```

## Usage Examples

### Vector Operations

```cpp
#include <microla/microla.hpp>

using namespace microla;

Vec<float, 3> a(1.0f, 0.0f, 0.0f);
Vec<float, 3> b(0.0f, 1.0f, 0.0f);

// Basic operations
Vec<float, 3> sum = a + b;              // (1, 1, 0)
float dot = a.dot(b);                   // 0.0
Vec<float, 3> cross = a.cross(b);       // (0, 0, 1)

// Advanced operations
Vec<float, 3> normalized = a.normalized();
float angle = a.angle(b);               // π/2 radians
Vec<float, 3> proj = a.project(b);

// Static factory methods
Vec<float, 3> zero = Vec<float, 3>::zero();
Vec<float, 3> unit_x = Vec<float, 3>::unit_x();

// Swizzling
Vec<float, 2> xy = a.xy();              // (1, 0)
Vec<float, 3> xyz = sum.xyz();          // (1, 1, 0)

// Interpolation
Vec<float, 3> mid = a.lerp(b, 0.5f);
Vec<float, 3> smooth = a.cubic_hermite(b, Vec<float,3>::zero(), Vec<float,3>::zero(), 0.5f);

// Clamping
Vec<float, 3> clamped = sum.clamped(-1.0f, 1.0f);
Vec<float, 3> saturated = sum.saturated();  // [0, 1]
```

### Matrix Operations

```cpp
#include <microla/microla.hpp>

using namespace microla;

// 3x3 rotation matrix
SquareMat<float, 3> R = SquareMat<float, 3>::rotation_z(1.57f);

// Matrix operations
SquareMat<float, 3> RT = R.transpose();
float det = R.determinant();
SquareMat<float, 3> inv = R.inverse();

// LU decomposition
auto [L, U, P] = R.lu();

// Embedded-friendly eigenvalues API (caller-owned buffer)
float eigenvalues[3] = {};
if (R.eigenvalues_qr(eigenvalues, 3)) {
  // eigenvalues now contains the QR result
}

// Transform vector
Vec<float, 3> v(1.0f, 0.0f, 0.0f);
Vec<float, 3> rotated = R * v;
```

### Quaternion Rotations

```cpp
#include <microla/quaternion.hpp>

using namespace microla;

// Create rotation quaternion
Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
Quaternion<float> q(axis, 1.57f);       // 90° around Z

// Rotate vector
Vec<float, 3> point(1.0f, 0.0f, 0.0f);
Vec<float, 3> rotated = q.rotate(point); // (0, 1, 0)

// Quaternion interpolation
Quaternion<float> q1 = Quaternion<float>::identity();
Quaternion<float> q2(axis, 3.14f);
Quaternion<float> mid = q1.slerp(q2, 0.5f);

// Euler angle conversion
Vec<float, 3> euler = q.to_euler();     // (roll, pitch, yaw)
Quaternion<float> q3 = Quaternion<float>::from_euler(0.1f, 0.2f, 0.3f);
float roll = q.roll();
float pitch = q.pitch();
float yaw = q.yaw();
```

### Mathematical Utilities

```cpp
#include <microla/microla.hpp>

using namespace microla;

// Constants
float pi = constants::pi<float>;
float deg90 = deg_to_rad(90.0f);        // Convert to radians
float rad90 = rad_to_deg(1.57f);        // Convert to degrees

// Angle utilities
float wrapped = wrap_pi(3.5f);          // Wrap to [-π, π]
float wrapped2 = wrap_two_pi(3.5f);     // Wrap to [0, 2π]
float shortest = angle_distance(0.1f, 6.2f);  // Shortest angular distance

// Clamping
float val = clamp(1.5f, 0.0f, 1.0f);    // 1.0
float sat = saturate(1.5f);             // 1.0 (clamp to [0,1])

// Homogeneous coordinates
Vec<float, 3> v3(1.0f, 2.0f, 3.0f);
Vec<float, 4> v4 = v3.to_homogeneous();  // (1, 2, 3, 1)
Vec<float, 3> back = v4.from_homogeneous();  // Perspective division
```

### Compile-Time Rotations

```cpp
#include <microla/microla.hpp>

using namespace microla;

// C++20: precompute fixed-orientation matrices once and reuse them
static const auto sensor_to_body = SquareMat<float, 3>::rotation_z(deg_to_rad(90.0f));
Vec<float, 3> sensor(1.0f, 0.0f, 0.0f);
Vec<float, 3> body = sensor_to_body * sensor;

// C++26: ANY angle at compile time (constexpr sin/cos)
#if __cplusplus >= 202600L
constexpr auto R = SquareMat<float, 3>::rotation_z(1.234f);  // Any angle!
#endif

// See docs/Cpp_Standard_Optimizations.md for details and workarounds
```

### Kalman Filtering

```cpp
#include <microla/kalman.hpp>
#include <microla/extended_kalman.hpp>

using namespace microla;

// Standard Kalman Filter - 1D position tracking
KalmanFilter<float, 2, 1> kf;  // 2 states (pos, vel), 1 measurement

// Configure state transition: x_k = F * x_{k-1}
auto F = Mat<float, 2, 2>::identity();
F(0, 1) = dt;  // Position += velocity * dt
kf.set_state_transition(F);

// Configure measurement matrix: z = H * x
auto H = Mat<float, 1, 2>();
H(0, 0) = 1.0f;  // Measure position only
kf.set_measurement_matrix(H);

// Configure noise covariances
kf.set_process_noise(Mat<float, 2, 2>::identity() * 0.01f);
auto R = Mat<float, 1, 1>();
R(0, 0) = 1.0f;
kf.set_measurement_noise(R);

kf.predict();
Vec<float, 1> measurement(sensor_reading);
kf.update(measurement);
float estimated_position = kf.get_state(0);

// Extended Kalman Filter - nonlinear systems with Jacobians
// Define function pointers for state transition and measurement
auto state_func = [](const Vec<float, 2>& x, float dt) -> Vec<float, 2> {
    return x;  // Replace with nonlinear f(x, dt)
};
auto meas_func = [](const Vec<float, 2>& x) -> Vec<float, 1> {
    return Vec<float, 1>(x[0]);  // Replace with nonlinear h(x)
};
auto state_jac = [](const Vec<float, 2>& x, float dt) -> Mat<float, 2, 2> {
    return Mat<float, 2, 2>::identity();  // Replace with df/dx
};
auto meas_jac = [](const Vec<float, 2>& x) -> Mat<float, 1, 2> {
    auto H = Mat<float, 1, 2>(); H(0,0) = 1.0f; return H;  // Replace with dh/dx
};

ExtendedKalmanFilter<float, 2, 1> ekf(
    state_func, meas_func, state_jac, meas_jac,
    Mat<float, 2, 2>::identity() * 0.01f,  // Q
    Mat<float, 1, 1>::identity() * 0.1f    // R
);

ekf.predict(dt);
ekf.update(measurement);
```

### Safe Math (Safety-Critical Systems)

```cpp
#include <microla/safe_math.hpp>

using namespace microla;

// Safe division with fallback
float result = safe::safe_divide(numerator, denominator);  // Returns 0 if div-by-zero
float result2 = safe::safe_divide(numerator, denominator, -1.0f);  // Custom fallback

// Saturating arithmetic (prevents overflow/underflow)
int safe_sum = safe::saturating_add(INT_MAX, 100);  // Saturates at INT_MAX
int safe_diff = safe::saturating_sub(INT_MIN, 100);  // Saturates at INT_MIN
int safe_prod = safe::saturating_mul(10000, 10000);  // Saturates instead of overflow

// Safe trigonometric functions (clamp inputs to valid range)
float angle = safe::safe_acos(1.5f);   // Clamps to 1.0, returns acos(1.0) = 0
float root = safe::safe_sqrt(-1.0f);   // Returns 0 (negative clamped to 0)
```

### Fast Math (High-Performance Approximations)

```cpp
#include <microla/fast_math.hpp>

using namespace microla::fast;

// Bhaskara I sine approximation (~0.17% error, 3-4× faster)
float s = sin(angle);
float c = cos(angle);

// Quake III fast inverse square root (2-3× faster)
float inv_len = rsqrt(x*x + y*y + z*z);

// Fast polynomial approximations
float e = exp(x);      // 2-3× faster
float l = ln(x);       // 2-3× faster
float p = pow(base, exponent);  // 4-5× faster

// Smooth interpolation
float lerp_val = lerp(a, b, t);
float smooth_val = smoothstep(edge0, edge1, x);  // Hermite interpolation
```

### Numerical Stability

```cpp
#include <microla/numerical_stability.hpp>

using namespace microla;

// Kahan summation for accurate floating-point sums
std::vector<float> values = {1e10f, 1.0f, 1.0f, 1.0f};
float accurate_sum = kahan_sum<float>(values.begin(), values.end());

// Stable hypot (avoids overflow for large values)
float magnitude = hypot(x, y);  // More stable than sqrt(x*x + y*y)
float magnitude3d = hypot3(x, y, z);

// Approximate equality with tolerance
if (approx_equal(a, b)) {
    // Values are approximately equal
}

// Condition number for matrix ill-conditioning detection
float cond = condition_number_2x2(a11, a12, a21, a22);
```

## Examples

The `examples/` directory contains practical demonstrations:

### Basic Usage (`basic_usage.cpp`)

Fundamental operations: vectors, matrices, quaternions, transformations.

### Sensor Fusion (`sensor_fusion.cpp`)

Relative angle estimation between mixed sensor packages using the public `microla::fusion` API:

```cpp
using JointEstimator = microla::fusion::RelativeAngleEstimator<
  float,
  AnchorImu9Config,
  FollowerImu6Config,
  JointAngleConfig>;

JointEstimator estimator;
estimator.update_left(anchor_sample);
estimator.update_right(follower_sample);
auto hinge = estimator.compute();
auto tilt = estimator.compute_scalar(microla::fusion::PrimaryScalarOutput::tilt_angle);
```

The maintained example shows compile-time policy selection, neutral-pose capture, drift-aware heading output, and runtime scalar-output override. See [docs/SENSOR_FUSION.md](docs/SENSOR_FUSION.md) for the full configuration model.

### Kalman Filtering (`kalman_demo.cpp`)

State estimation with Standard and Extended Kalman Filters:
- 1D position/velocity tracking
- 2D range/bearing measurements
- Sensor fusion with multiple measurement sources

### Robotics Kinematics (`robotics_kinematics.cpp`)

2-link planar robot arm with forward/inverse kinematics, Jacobian, and trajectory planning.

### Graphics Pipeline (`graphics_pipeline.cpp`)

3D graphics transformations: model-view-projection matrices, viewport transforms, frustum culling.

### Compile-Time Rotations (`constexpr_rotations.cpp`)

Demonstrates constexpr factory methods and compile-time matrix operations.

### Advanced Features (`advanced_features_demo.cpp`)

SIMD optimizations, compile-time checks, and performance patterns.

**Build examples:**

```bash
cmake --preset debug
cmake --build --preset debug
./build/debug/examples/sensor_fusion
./build/debug/examples/kalman_demo
```

## Performance Benchmarks

The `benchmarks/` directory provides Google Benchmark-based performance tests:

- **Matrix Operations**: Multiplication, transpose, determinant, inverse
- **SIMD Comparisons**: Vec3/Vec4/Quaternion operations with/without SIMD
- **Constexpr vs Runtime**: Compile-time initialization performance
- **Vector Operations**: Dot, cross, normalize, interpolation, batch processing
- **Sensor Fusion**: Mahony, MEKF, calibration, and relative-angle pipeline costs

**Run benchmarks:**

```bash
cmake --preset benchmark
cmake --build --preset benchmark
./build/benchmark-native/benchmarks/bench_matrix_multiply
./build/benchmark-native/benchmarks/bench_simd --benchmark_filter=Vec3.*
./build/benchmark-native/benchmarks/bench_sensor_fusion --benchmark_filter=bm_sensor_fusion_.*
```

See [benchmarks/README.md](benchmarks/README.md) for detailed results and optimization tips.

For detailed performance comparisons with other libraries, see [PERFORMANCE.md](PERFORMANCE.md).

## Documentation

- **[Quick Reference](QUICK_REFERENCE.md)** - Compact API cheat sheet
- **[API Documentation](docs/API_Documentation.md)** - Complete API reference
- **[Migration Guide](MIGRATION.md)** - Switch from Eigen, GLM, or custom code
- **[Performance Comparison](PERFORMANCE.md)** - Benchmarks vs. alternatives
- **[C++ Optimizations](docs/Cpp_Standard_Optimizations.md)** - C++20-26 feature usage
- **[Doxygen](docs/doxygen/html/index.html)** - Generated API docs (run `bash scripts/generate_docs.sh`)
- **[Cookbook](docs/COOKBOOK.md)** - Practical recipes and extended Kalman filter examples
- **[Sensor Fusion Guide](docs/SENSOR_FUSION.md)** - IMU orientation estimation
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions

## SIMD Optimizations

### ARM NEON (Cortex-A, ARM64, Apple Silicon)

Enable NEON optimizations for high-performance ARM processors:

```cmake
# In CMakeLists.txt
cmake -DMICROLA_ENABLE_NEON=ON ..
```

Optimized operations for `Vec<float, 2/3/4>` and `Quaternion<float>`:
- Addition/subtraction: `vaddq_f32`, `vsubq_f32`
- Scalar multiplication: `vmulq_f32`
- Dot product: `vmulq_f32` + horizontal reduction
- Negation: `vnegq_f32`

### CMSIS-DSP (Cortex-M)

Enable hardware acceleration on ARM Cortex-M4F/M7/M33/M85 processors:

```cmake
# In CMakeLists.txt
target_compile_definitions(your_target PRIVATE CONFIG_MICROLA_CMSIS)
target_link_libraries(your_target PRIVATE CMSIS::DSP)

# Or manually
target_link_libraries(your_target PRIVATE arm_cortexM_math)
```

Comprehensive CMSIS-DSP optimizations for `float` types:

**Vector operations:**
- Addition/subtraction: `arm_add_f32`, `arm_sub_f32`
- Scalar operations: `arm_scale_f32`, `arm_negate_f32`
- Dot product: `arm_dot_prod_f32`
- Normalization: `arm_sqrt_f32` + `arm_scale_f32`

**Matrix operations:**
- Matrix arithmetic: `arm_mat_add_f32`, `arm_mat_sub_f32`, `arm_mat_scale_f32`
- Matrix multiplication: `arm_mat_mult_f32`
- Transpose: `arm_mat_trans_f32`
- Matrix-vector: Uses `arm_mat_mult_f32` with column vector

**Quaternion operations:**
- Normalization: `arm_sqrt_f32` + `arm_scale_f32`
- Conjugate: `arm_negate_f32` (imaginary parts)
- Norm calculation: `arm_dot_prod_f32`

**Performance benefits:**
- Cortex-M7: 1.8-3.0× speedup (matrix ops best)
- Cortex-M4F: 1.3-2.0× speedup
- Power efficient: Hardware DSP instructions, lower energy consumption
- Optimized for: Motor control, IMU fusion, robotics, audio processing

**Performance**: Three-tier optimization strategy (NEON → CMSIS-DSP → Generic) provides optimal performance across the entire ARM ecosystem.

## Additional Documentation

- [API Documentation](docs/API_Documentation.md) - Complete API reference with examples
- [C++ Standard Optimizations](docs/Cpp_Standard_Optimizations.md) - C++20/23/26 feature usage and performance
- [Troubleshooting](docs/TROUBLESHOOTING.md) - build issues, sanitizer setup, and integration notes
- [Examples](examples/) - Code examples and usage patterns

## Library Components

All classes are in the `microla` namespace:

### Header Organization

The library uses a modular header structure for flexibility:

- **microla.hpp** - Main convenience header that includes all components
- **compiler_features.hpp** - C++ standard feature detection (C++20-C++26 feature macros)
- **constants.hpp** - Mathematical constants (pi, e, sqrt2, etc.) - independent, can be used standalone
- **vector.hpp** - Generic `Vec<T,N>` template with all vector operations
- **matrix.hpp** - Generic `Mat<T,R,C>` and `SquareMat<T,N>` templates
- **quaternion.hpp** - Quaternion rotations with [x,y,z,w] memory layout
- **geometry.hpp** - Geometric primitives (Ray, Plane, AABB, Sphere, Triangle, Frustum)
- **kalman.hpp** - Standard Kalman filter
- **extended_kalman.hpp** - Extended Kalman filter with Jacobian linearization
- **safe_math.hpp** - Safety-critical operations (safe_divide, safe_sqrt, saturating arithmetic)
- **fast_math.hpp** - Fast approximations (sin, cos, rsqrt, exp, ln)
- **numerical_stability.hpp** - Kahan summation, stable hypot, condition numbers
- **resource_checks.hpp** - Compile-time size/layout verification
- **matrix_view.hpp** - Non-owning matrix slices
- **vector_view.hpp** - Non-owning vector slices with stride support
- **simd_helpers.hpp** - Platform-agnostic SIMD helper utilities
- **high_precision.hpp** - High-precision arithmetic (compensated sum, error-free transforms)
- **version.hpp** - Runtime version API

### Vec<T, N>

- N-dimensional vectors with type T
- Common indices: `X=0, Y=1, Z=2, W=3`
- Optimized for 2D, 3D, and 4D operations
- NEON-accelerated for `Vec<float, 2/3/4>`

### Mat<T, R, C>

- R×C matrices with row-major storage
- Generic matrix operations
- CMSIS-DSP optimized for `float` types

### SquareMat<T, N>

- Square matrices with additional operations
- Rotation, operator* for scale, translation matrices
- Determinant, inverse, eigenvalues

### Quaternion<T>

- Efficient 3D rotation representation
- Memory layout optimized for Vec<T,3> operations
- SLERP interpolation for smooth animations
- NEON-accelerated for `Quaternion<float>`

### Version API

- Runtime version checking: `get_version_string()`, `get_version_number()`
- Compatibility checking: `version_at_least(major, minor, patch)`

## Requirements

- **C++ Standard**: C++20 or later
- **Dependencies**: None (NEON/CMSIS-DSP optional for ARM targets)
- **Compiler Support**: GCC, Clang, MSVC, ARM Compiler 6, IAR Embedded Workbench for ARM
- **Tested Platforms**:
  - ARM Cortex-A (with NEON)
  - ARM64/AArch64 (with NEON)
  - Apple Silicon M1/M2/M3 (with NEON)
  - ARM Cortex-M0/M0+/M3/M4/M7/M33 (with CMSIS-DSP)
  - x86/x64 desktop platforms
  - Zephyr RTOS 3.0+

## Performance

All types are POD (Plain Old Data) with the following characteristics:
- Zero runtime overhead for abstractions
- Inline-friendly for compiler optimization
- Cache-friendly contiguous memory layout
- 16-byte alignment for SIMD operations
- No virtual functions or dynamic dispatch

## Building Examples and Tests

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

## Integration with Zephyr

MicroLA integrates seamlessly with Zephyr RTOS:

```c
// prj.conf
CONFIG_MICROLA_LINEAR=y
CONFIG_MICROLA_NEON=y   # Optional: Enable NEON (Cortex-A/ARM64)
CONFIG_MICROLA_CMSIS=y  # Optional: Enable CMSIS-DSP (Cortex-M)
CONFIG_MICROLA_MVE=y    # Optional: Enable MVE/Helium (Cortex-M55/M85)

// CMakeLists.txt
target_link_libraries(app PUBLIC microla)
```

## Testing

### Unit Tests

MicroLA includes comprehensive test suites:

**Google Test Suite** (for desktop/CI):

```bash
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests
```

Tests include:
- Compile-time constexpr validation (C++20+)
- Vector/matrix/quaternion operations
- Edge cases and numerical stability
- Type conversions and utilities
- SIMD correctness

**Zephyr Test Suite** (for embedded targets):

```bash
west build -b nrf52840dk_nrf52840 tests/zephyr
west flash
```

### Sanitizers

MicroLA is exercised under host sanitizers in CI:

```bash
cmake --preset sanitizers-ci
cmake --build --preset sanitizers-ci
ctest --preset sanitizers-ci
```

Passes in repository CI: AddressSanitizer (ASan) and UndefinedBehaviorSanitizer (UBSan)

MemorySanitizer is not currently part of the checked-in CI workflow.

### Development Checks

The repository ships helper scripts for the local formatting and clang-tidy flows used in CI:

```bash
./scripts/format.sh --check
cmake --preset static-analysis
BUILD_DIR=build/static-analysis \
TIDY_INCLUDE_REGEX='.*/examples/.*[.](cpp|c)$' \
./scripts/tidy.sh --config-file="$PWD/.clang-tidy" --header-filter='^$' --warnings-as-errors='*'
```

## Using with IAR Embedded Workbench

MicroLA is fully compatible with IAR Embedded Workbench for ARM:

**Quick Setup:**
1. Add `microla/include` to your project's include paths
2. Enable C++20: Project Options → C/C++ Compiler → Language → C++ → C++20
3. For NEON optimization: Project Options → C/C++ Compiler → Code → FPU: VFPv4_sp
4. Add preprocessor define: `CONFIG_MICROLA_NEON`

**Example:**

```cpp
#include <microla/microla.hpp>
#include <microla/quaternion.hpp>

using namespace microla;

void compute_rotation() {
    Vec<float, 3> axis(0, 0, 1);
    Quaternion<float> q(axis, 1.57f);
    Vec<float, 3> point(1, 0, 0);
    Vec<float, 3> result = q.rotate(point);
}
```

See [QUICKSTART.md](QUICKSTART.md) for detailed IAR integration instructions.

## AI-Assisted Development

This library was developed with assistance from **GitHub Copilot** using the **Claude Sonnet 4.5** model. The AI helped with:
- Code generation and optimization
- Documentation and comments
- SIMD optimization implementation (NEON/CMSIS-DSP)
- Test coverage and examples

See [.ai-generation-prompt.md](.ai-generation-prompt.md) for the regeneration prompt.

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE) for full details.

### AI Generation Disclosure

This library was substantially generated with AI coding assistance. Copyright is asserted by
James Baldwin in the creative selection, arrangement, specification, and curation of the work.
No warranty is made regarding originality or non-infringement of AI-generated portions.
See [NOTICE](NOTICE) for the full intellectual property disclosure.

For safety-critical or regulated applications (IEC 61508, ISO 26262, DO-178C, FDA), users
should independently audit the codebase and consult qualified IP counsel before deployment.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Authors

- Created for embedded systems and robotics applications
- Optimized for ARM Cortex-M microcontrollers
- Designed with real-time constraints in mind

## Version

Current Version: 0.0.3
