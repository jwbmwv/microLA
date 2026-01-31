# MicroLA

A lightweight, header-only C++11 linear algebra library optimized for embedded systems and real-time applications.

## Features

- **Header-only**: No compilation required, just include the headers
- **Zero allocation**: All operations use stack memory, no dynamic allocation
- **POD types**: All classes are Plain Old Data compatible with C interfaces and DMA
- **Default template parameters**: Use `Vec<>`, `Mat<>`, `Quaternion<>` for common float types
- **SIMD optimized**: ARM NEON (2-4× speedup) for Cortex-A/ARM64, CMSIS-DSP for Cortex-M processors, RISC-V V extensions for RISC-V architectures
  - **FMA instructions**: Fused multiply-add for better performance and accuracy
  - **Fast reciprocal/rsqrt**: Newton-Raphson optimized division and normalize
  - **Vec3 optimized**: Eliminated temporary arrays, 1.4× faster
  - **NEON quaternion multiply**: 3.5× faster than scalar
  - **Matrix SIMD**: 3-4× speedup for 3×3 and 4×4 operations
- **C++ standard adaptive**: Progressive optimizations C++11/14/17/20/23/26, baseline C++11
- **Compile-time rotations**: C++26 constexpr trig, special angles for C++11-C++23
- **Namespaced**: All classes in `microla` namespace to avoid pollution
- **Versioned**: Runtime version API for compatibility checking
- **Type-safe**: Compile-time dimension checking prevents errors
- **Const-correct**: Full const correctness throughout the API
- **Sanitizer-clean**: Zero UB, passes ASan/UBSan/MSan without errors
- **Embedded-friendly**: Designed for microcontrollers with limited resources
- **Rich utilities**: Mathematical constants, angle utilities, interpolation, swizzling
- **Sensor fusion ready**: Coordinate transforms, safe normalization, Euler/quaternion conversions
- **Kalman filtering**: Standard and Extended Kalman Filters for state estimation
- **Safe math**: Protected operations with overflow/underflow detection and clamping
- **Fast math**: High-performance approximations (2-10× speedup) for embedded systems
- **Numerical stability**: Kahan summation, stable hypot, condition number checks

## Quick Start

### As a Git Submodule

```bash
# Add to your project
git submodule add https://github.com/yourusername/microla.git external/microla
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
      url: https://github.com/yourusername/microla.git
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
#include <microla/vec2D.hpp>         // Vec2f, Vec2d type aliases
#include <microla/vec3D.hpp>         // Vec3f, Vec3d type aliases
#include <microla/matrix.hpp>        // Base Mat<T,R,C> templates
#include <microla/matrix2D.hpp>      // 2D transformations
#include <microla/matrix3D.hpp>      // 3D/4D transformations

using namespace microla;

Vec<float, 3> v(1.0f, 2.0f, 3.0f);
Quaternion<float> q(Vec<float, 3>(0, 0, 1), 1.57f);

// Check library version
if (version_at_least(1, 0, 0)) {
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

// Eigenvalues
Vec<float, 3> eigenvalues = R.eigenvaluesQR();

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

// C++11-C++23: Special angles (0°, 90°, 180°, 270°) at compile time
constexpr auto R90z = SquareMat<float, 3>::rotation_z_deg<90>();
constexpr auto R180x = SquareMat<float, 3>::rotation_x_deg<180>();
constexpr auto R270y = SquareMat<float, 3>::rotation_y_deg<270>();

// Perfect for fixed sensor orientations - zero runtime cost!
constexpr Vec<float, 3> sensor(1.0f, 0.0f, 0.0f);
constexpr Vec<float, 3> body = R90z * sensor;  // Computed at compile time

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
kf.F = Mat<float, 2, 2>::identity();  // State transition
kf.F(0, 1) = dt;  // Position += velocity * dt
kf.H(0, 0) = 1.0f;  // Measure position
kf.Q = Mat<float, 2, 2>::identity() * 0.01f;  // Process noise
kf.R(0, 0) = 1.0f;  // Measurement noise

kf.predict();
Vec<float, 1> measurement(sensor_reading);
kf.update(measurement);
float estimated_position = kf.x[0];

// Extended Kalman Filter - nonlinear systems with Jacobians
ExtendedKalmanFilter<float, 2, 1> ekf;
ekf.state_model = [](const Vec<float, 2>& x) { /* nonlinear f(x) */ };
ekf.state_jacobian = [](const Vec<float, 2>& x) { /* df/dx */ };
ekf.measurement_model = [](const Vec<float, 2>& x) { /* nonlinear h(x) */ };
ekf.measurement_jacobian = [](const Vec<float, 2>& x) { /* dh/dx */ };

ekf.predict();
ekf.update(measurement);
```

### Safe Math (Safety-Critical Systems)

```cpp
#include <microla/safe_math.hpp>

using namespace microla;

// Safe division with fallback
float result;
if (safe_divide(numerator, denominator, result)) {
    // Division succeeded
} else {
    // Division by zero avoided, result = fallback (default 0)
}

// Saturating arithmetic (prevents overflow/underflow)
int safe_sum = saturating_add(INT_MAX, 100);  // Saturates at INT_MAX
int safe_diff = saturating_sub(INT_MIN, 100);  // Saturates at INT_MIN
int safe_prod = saturating_mul(10000, 10000);  // Saturates instead of overflow

// Safe trigonometric functions (clamp inputs to valid range)
safe_acos(1.5f, result);  // Clamps to 1.0, returns acos(1.0)
safe_sqrt(-1.0f, result, fallback_value);  // Returns false, result = fallback
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
float accurate_sum = kahan_sum(values.data(), values.size());

// Stable hypot (avoids overflow for large values)
float magnitude = hypot(x, y);  // More stable than sqrt(x*x + y*y)
float magnitude3d = hypot3(x, y, z);

// Safe normalization with length check
if (safe_normalize(x, y, z)) {
    // Normalized successfully, (x, y, z) is now unit length
}

// Condition number for matrix ill-conditioning detection
float cond = condition_number_2x2(a11, a12, a21, a22);
if (is_ill_conditioned(a11, a12, a21, a22)) {
    // Matrix is nearly singular, numerical issues likely
}
```

## Examples

The `examples/` directory contains practical demonstrations:

### Basic Usage (`basic_usage.cpp`)
Fundamental operations: vectors, matrices, quaternions, transformations.

### Sensor Fusion (`sensor_fusion.cpp`)
IMU sensor fusion using complementary filter with quaternion orientation estimation:
```cpp
ComplementaryFilter filter(0.98f);
filter.update(gyro, accel, dt);
Vec3f euler = filter.get_euler_angles();
```

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
cmake .. -DMICROLA_LINEAR_BUILD_EXAMPLES=ON
make
./examples/sensor_fusion
./examples/kalman_demo
```

## Performance Benchmarks

The `benchmarks/` directory provides Google Benchmark-based performance tests:

- **Matrix Operations**: Multiplication, transpose, determinant, inverse
- **SIMD Comparisons**: Vec3/Vec4/Quaternion operations with/without SIMD
- **Constexpr vs Runtime**: Compile-time initialization performance
- **Vector Operations**: Dot, cross, normalize, interpolation, batch processing

**Run benchmarks:**
```bash
cmake .. -DMICROLA_LINEAR_BUILD_BENCHMARKS=ON
make
./benchmarks/bench_matrix_multiply
./benchmarks/bench_simd --benchmark_filter=Vec3.*
```

See [benchmarks/README.md](benchmarks/README.md) for detailed results and optimization tips.

For detailed performance comparisons with other libraries, see [PERFORMANCE.md](PERFORMANCE.md).

## Documentation

- **[Quick Reference](QUICK_REFERENCE.md)** - Compact API cheat sheet
- **[API Documentation](docs/API_Documentation.md)** - Complete API reference
- **[Migration Guide](MIGRATION.md)** - Switch from Eigen, GLM, or custom code
- **[Performance Comparison](PERFORMANCE.md)** - Benchmarks vs. alternatives
- **[C++ Optimizations](docs/Cpp_Standard_Optimizations.md)** - C++11-26 feature usage
- **[Doxygen](docs/doxygen/html/index.html)** - Generated API docs (run `doxygen`)

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

## Documentation

- [API Documentation](docs/API_Documentation.md) - Complete API reference with examples
- [C++ Standard Optimizations](docs/Cpp_Standard_Optimizations.md) - C++14/17/20 features and performance
- [Sanitizer Safety](docs/Sanitizer_Safety.md) - UB prevention, memory safety, and sanitizer testing
- [Examples](examples/) - Code examples and usage patterns

## Library Components

All classes are in the `microla` namespace:

### Header Organization

The library uses a modular header structure for flexibility:

- **microla.hpp** - Main convenience header that includes all components
- **compiler_features.hpp** - C++ standard feature detection (C++11-C++26 compatibility macros)
- **constants.hpp** - Mathematical constants (pi, e, sqrt2, etc.) - independent, can be used standalone
- **vector.hpp** - Generic `Vec<T,N>` template with all vector operations
- **vec2D.hpp** - 2D vector type aliases (`Vec2f`, `Vec2d`, etc.)
- **vec3D.hpp** - 3D vector type aliases (`Vec3f`, `Vec3d`, etc.)
- **matrix.hpp** - Generic `Mat<T,R,C>` and `SquareMat<T,N>` templates
- **matrix2D.hpp** - 2D transformation matrices (rotation, operator* for scale)
- **matrix3D.hpp** - 3D/4D transformation matrices (rotations, look-at, translation)
- **quaternion.hpp** - Quaternion rotations
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

- **C++ Standard**: C++11 or later
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
mkdir build && cd build
cmake ..
cmake --build .

# Run tests
ctest --output-on-failure
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
cmake .. -DMICROLA_LINEAR_BUILD_TESTS=ON
make
./tests/google/microla_gtests
```

Tests include:
- Compile-time constexpr validation (C++14+)
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

MicroLA is sanitizer-clean:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"
make && ./tests/google/microla_gtests
```

Passes: AddressSanitizer (ASan), UndefinedBehaviorSanitizer (UBSan), MemorySanitizer (MSan)

## Using with IAR Embedded Workbench

MicroLA is fully compatible with IAR Embedded Workbench for ARM:

**Quick Setup:**
1. Add `microla/include` to your project's include paths
2. Enable C++11: Project Options → C/C++ Compiler → Language → C++ → C++11
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

MIT License - see [LICENSE](LICENSE) file for details

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Authors

- Created for embedded systems and robotics applications
- Optimized for ARM Cortex-M microcontrollers
- Designed with real-time constraints in mind

## Version

Current Version: 1.0.0
