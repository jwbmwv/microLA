# MicroLA Quick Reference

A compact reference for common operations. See [API_Documentation.md](docs/API_Documentation.md) for complete details.

## Quick Links

- [Vectors](#vectors) | [Matrices](#matrices) | [Quaternions](#quaternions) | [Constants](#constants) | [Utilities](#utilities)
- [Examples](examples/) | [Benchmarks](benchmarks/) | [Tests](tests/)

---

## Vectors

```cpp
#include <microla/microla.hpp>
using namespace microla;

// Construction
Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
Vec3f v2{1.0f, 2.0f, 3.0f};          // Type alias
auto v3 = Vec3f::zero();              // Factory (constexpr)
auto v4 = Vec3f::one();

// Access
float x = v1[0];                      // Operator[]
float y = v1.x();                     // Named accessor (Vec2/3 only)

// Arithmetic
Vec3f sum = v1 + v2;
Vec3f diff = v1 - v2;
Vec3f scaled = v1 * 2.0f;
Vec3f neg = -v1;

// Vector Operations
float dot = v1.dot(v2);
Vec3f cross = v1.cross(v2);           // 3D only
float len = v1.length();
float len2 = v1.length_squared();     // Faster (no sqrt)
Vec3f unit = v1.normalized();

// Interpolation
Vec3f lerp = v1.lerp(v2, 0.5f);      // Linear
Vec3f slerp = v1.slerp(v2, 0.5f);    // Spherical (normalized)

// Utilities
float dist = v1.distance(v2);
Vec3f reflected = v1.reflect(normal);
Vec3f projected = v1.project(onto);
Vec3f clamped = v1.clamp(min, max);

// Swizzle (compile-time indices)
Vec3f swz = v1.swizzle<2, 1, 0>();   // z, y, x
```

---

## Matrices

```cpp
#include <microla/microla.hpp>
using namespace microla;

// Construction
Mat<float, 3, 3> m1;                  // Uninitialized
SquareMat<float, 3> m2 = SquareMat<float, 3>::identity();  // constexpr
auto m3 = Mat3f::zero();
Mat4f m4{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

// Access
float val = m1(row, col);             // Operator()
m1(1, 2) = 5.0f;                      // Assignment

// Arithmetic
Mat3f sum = m1 + m2;
Mat3f product = m1 * m2;              // Matrix multiply
Vec3f transformed = m1 * v1;          // Matrix-vector multiply
Mat3f scaled = m1 * 2.0f;

// Matrix Operations
Mat3f transposed = m1.transpose();
float det = m2.determinant();         // Square matrices
Mat3f inv = m2.inverse();             // Square matrices
float trace = m2.trace();             // Square matrices
uint32_t rank = m2.rank();

// 2D Transformations
Mat2f rot = Mat2f::rotation(angle_rad);
Mat2f rot90 = Mat2f::rotation_deg<90>();  // constexpr
Mat2f scale = Mat2f::scale(Vec2f{2.0f, 3.0f});

// 3D Transformations
Mat3f rotX = SquareMat<float,3>::rotation_x(angle);
Mat3f rotY = SquareMat<float,3>::rotation_y(angle);
Mat3f rotZ = SquareMat<float,3>::rotation_z(angle);
Mat3f rot_axis = SquareMat<float,3>::rotation(angle, axis);

// 3D Scaling
Mat3f scale3 = SquareMat<float,3>::scale(Vec3f{2,2,2});

// 4x4 Transformations
Mat4f view = Mat4f::look_at(eye, target, up);
Mat4f proj = Mat4f::perspective(fov, aspect, near, far);
Mat4f ortho = Mat4f::orthographic(l, r, b, t, n, f);
Mat4f trans = Mat4f::translation(Vec3f{x, y, z});
```

---

## Quaternions

```cpp
#include <microla/quaternion.hpp>
using namespace microla;

// Construction
Quaternion<float> q1 = Quaternion<float>::identity();  // constexpr
Quaternion<float> q2(axis, angle_rad);  // Axis-angle
Quaternion<float> q3 = Quaternion<float>::from_euler(roll, pitch, yaw);

// Access
float w = q1.w;                       // Public members: w, x, y, z
Vec3f vec = q1.vec();                 // Returns Vec3(x, y, z)
q1.set_vec(Vec3f{1, 0, 0});

// Arithmetic
Quaternion<float> prod = q1 * q2;     // Composition
Quaternion<float> conj = q1.conjugate();
Quaternion<float> inv = q1.inverse();
Quaternion<float> unit = q1.normalized();

// Rotation
Vec3f rotated = q1.rotate(point);
Vec3f rotated_inv = q1.rotate_inverse(point);

// Interpolation
Quaternion<float> lerp = q1.lerp(q2, t);
Quaternion<float> slerp = q1.slerp(q2, t);  // Spherical (better)

// Conversion
Vec3f euler = q1.to_euler();          // (roll, pitch, yaw)
Mat3f mat = q1.to_matrix();
float roll = q1.roll();
float pitch = q1.pitch();
float yaw = q1.yaw();

// Utilities
float dot = q1.dot(q2);
float angle = q1.angle();
Vec3f axis = q1.axis();
```

---

## Constants

```cpp
#include <microla/constants.hpp>
using namespace microla;

// Mathematical constants (template variables)
float pi = constants::pi<float>;
double pi_d = constants::pi<double>;

// Available constants:
constants::pi<T>             // π (3.14159...)
constants::two_pi<T>         // 2π
constants::half_pi<T>        // π/2
constants::quarter_pi<T>     // π/4
constants::inv_pi<T>         // 1/π
constants::e<T>              // e (2.71828...)
constants::sqrt2<T>          // √2
constants::sqrt3<T>          // √3
constants::inv_sqrt2<T>      // 1/√2
constants::inv_sqrt3<T>      // 1/√3
constants::golden_ratio<T>   // φ (1.61803...)
constants::deg_to_rad<T>     // π/180
constants::rad_to_deg<T>     // 180/π
```

---

## Utilities

```cpp
#include <microla/microla.hpp>
using namespace microla;

// Angle conversion
float rad = deg_to_rad(90.0f);        // Degrees → radians
float deg = rad_to_deg(1.57f);        // Radians → degrees

// Angle wrapping
float w1 = wrap_pi(angle);            // Wrap to [-π, π]
float w2 = wrap_two_pi(angle);        // Wrap to [0, 2π]
float dist = angle_distance(a1, a2);  // Shortest angular distance

// Clamping
float c = clamp(value, min, max);
float s = saturate(value);            // Clamp to [0, 1]

// Homogeneous coordinates
Vec4f h = v3.to_homogeneous();        // Append w=1
Vec3f back = h.from_homogeneous();    // Perspective divide

// Safe operations
bool is_norm = v.is_normalized(eps); // Check unit vector
Vec3f safe_norm = v.safe_normalized(fallback);  // Never NaN
```

---

## Compile-Time Operations

```cpp
// C++14+ constexpr factory methods
constexpr Vec3f zero = Vec3f::zero();
constexpr Mat4f identity = Mat4f::identity();
constexpr Quaternion<float> quat_id = Quaternion<float>::identity();

// C++11+ constexpr special angle rotations
constexpr Mat2f rot90 = Mat2f::rotation_deg<90>();
constexpr Mat3f rotX180 = SquareMat<float,3>::rotation_x_deg<180>();

// Lookup tables (zero runtime cost)
static constexpr Mat2f rotations[] = {
    Mat2f::rotation_deg<0>(),
    Mat2f::rotation_deg<90>(),
    Mat2f::rotation_deg<180>(),
    Mat2f::rotation_deg<270>()
};
```

---

## Type Aliases

```cpp
// 2D Vectors
Vec2<T>, Vec2f, Vec2d, Vec2i, Vec2u

// 3D Vectors
Vec3<T>, Vec3f, Vec3d, Vec3i, Vec3u

// Matrices
Mat2<T>, Mat2f, Mat2d           // 2×2
Mat3<T>, Mat3f, Mat3d           // 3×3
Mat4<T>, Mat4f, Mat4d           // 4×4

// Quaternions
Quaternion<float>, Quaternion<double>
```

---

## Build Options

```bash
# Examples
cmake .. -DMICROLA_LINEAR_BUILD_EXAMPLES=ON

# Tests
cmake .. -DMICROLA_LINEAR_BUILD_TESTS=ON

# Benchmarks
cmake .. -DMICROLA_LINEAR_BUILD_BENCHMARKS=ON

# SIMD Optimization

## Enable SIMD (all platforms)
```cpp
#define MICROLA_AUTODETECT_SIMD  // Auto-detect platform
```

## Platform-specific SIMD
```cpp
// ARM NEON (Cortex-A, ARM64)
#define CONFIG_MICROLA_NEON

// CMSIS-DSP (Cortex-M4F/M7)
#define CONFIG_MICROLA_CMSIS

// RISC-V V Extensions
#define CONFIG_MICROLA_RISCV
```
cmake .. -DMICROLA_ENABLE_NEON=ON      # ARM NEON
cmake .. -DMICROLA_ENABLE_CMSIS=ON     # ARM Cortex-M

# C++ Standard
cmake .. -DCMAKE_CXX_STANDARD=20

# Sanitizers
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"
```

---

## Performance Tips

1. **Use `length_squared()` instead of `length()` when possible** - Avoids expensive sqrt
2. **Prefer quaternions over Euler angles** - Faster, no gimbal lock
3. **Use constexpr for fixed transformations** - Zero runtime cost
4. **Enable SIMD for production** - 2-4× speedup for float operations
5. **Pass by const reference for large types** - Mat4f, SquareMat<T,4>
6. **Use `normalized()` judiciously** - Contains expensive sqrt + divisions

---

## Common Patterns

### Transform Point by Matrix
```cpp
Vec3f world_point = model_matrix * local_point;
```

### Transform Vector by Quaternion
```cpp
Vec3f rotated = orientation.rotate(vector);
```

### Build MVP Matrix (Graphics)
```cpp
Mat4f mvp = projection * view * model;
```

### Sensor Fusion (IMU)
```cpp
Quaternion<float> predicted = orientation * gyro_delta;
Quaternion<float> fused = predicted.slerp(accel_orientation, 0.02f);
```

### Forward Kinematics
```cpp
Mat3f shoulder = Mat3f::rotation_z(theta1);
Mat3f elbow = Mat3f::rotation_z(theta2);
Vec3f end_effector = shoulder * (link1 + elbow * link2);
```

---

## Updated Quick Reference for SquareMat

// Construction
SquareMat<float, 3> m2 = SquareMat<float, 3>::identity();  // Identity matrix
SquareMat<float, 3> m3 = SquareMat<float, 3>::rotation_z(1.57f);  // Rotation matrix

// LU Decomposition
auto [L, U, P] = m2.lu();

// Eigenvalues
Vec<float, 3> eigenvalues = m2.eigenvaluesQR();

// Matrix Operations
float trace = m2.trace();
SquareMat<float, 3> inv = m2.inverse();

---

**See [docs/API_Documentation.md](docs/API_Documentation.md) for complete API reference.**
