# Linear Algebra Library Documentation

A header-only C++17 linear algebra library providing efficient vector, matrix, and quaternion operations for embedded systems and real-time applications.

## Features

- **Header-only**: No compilation required, just include the headers
- **Zero allocation**: All operations use stack memory
- **POD types**: Compatible with C interfaces and DMA transfers
- **SIMD optimized**: ARM NEON for Cortex-A/ARM64, CMSIS-DSP for Cortex-M processors
- **Namespaced**: All classes in `microla` namespace
- **Versioned**: Runtime version API for compatibility checking
- **Type-safe**: Compile-time dimension checking
- **Const-correct**: Full const correctness throughout
- **Modular**: Include only what you need for faster compilation
- **Safety features**:
  - Bounds-checked `at()` accessors (debug mode only)
  - Division-by-zero prevention
  - Epsilon-based float comparisons
  - No undefined behavior

## Including the Library

### Option 1: All-in-One Include

```cpp
#include <microla/microla.hpp>   // Includes all vector/matrix components
#include <microla/quaternion.hpp>
#include <microla/version.hpp>

using namespace microla;
```

### Option 2: Modular Includes (Recommended for Faster Compilation)

```cpp
// Base templates
#include <microla/constants.hpp>     // Mathematical constants (can be used standalone)
#include <microla/vector.hpp>        // Generic Vec<T,N> template
#include <microla/matrix.hpp>        // Generic Mat<T,R,C> and SquareMat<T,N>

// Dimension-specific headers
#include <microla/vec2D.hpp>         // Vec2f, Vec2d type aliases
#include <microla/vec3D.hpp>         // Vec3f, Vec3d type aliases
#include <microla/matrix2D.hpp>      // 2D transformations (rotation, scale)
#include <microla/matrix3D.hpp>      // 3D/4D transformations (rotations, look-at, etc.)

// Other components
#include <microla/quaternion.hpp>
#include <microla/version.hpp>

using namespace microla;
```

### Header Organization

- **compiler_features.hpp** - C++ feature detection macros (MICROLA_CONSTEXPR, MICROLA_NODISCARD, MICROLA_BIT_CAST, etc.) - **independent**, provides centralized feature detection
- **microla.hpp** - Convenience header that includes all components and utility functions
- **constants.hpp** - Mathematical constants (pi, e, sqrt2, golden_ratio, etc.) with C++17-C++26 optimizations - **independent**, can be used standalone
- **vector.hpp** - Generic `Vec<T,N>` template with all vector operations
- **vec2D.hpp** - 2D vector type aliases (`Vec2<T>`, `Vec2f`, `Vec2d`, `Vec2i`, `Vec2u`)
- **vec3D.hpp** - 3D vector type aliases (`Vec3<T>`, `Vec3f`, `Vec3d`, `Vec3i`, `Vec3u`)
- **matrix.hpp** - Generic `Mat<T,R,C>` and `SquareMat<T,N>` templates
- **matrix2D.hpp** - 2D transformation matrices (`SquareMat<T,2>` specializations)
- **matrix3D.hpp** - 3D/4D transformation matrices (`SquareMat<T,3>` and `SquareMat<T,4>` specializations)
- **quaternion.hpp** - Quaternion rotations
- **version.hpp** - Runtime version API

## Table of Contents

- [Constants Namespace](#constants-namespace)
- [Vec Class](#vec-class)
- [Mat Class](#mat-class)
- [Quaternion Class](#quaternion-class)
- [Version API](#version-api)
- [SIMD Optimizations](#simd-optimizations)
- [Common Patterns](#common-patterns)

---

## Constants Namespace

The `constants` namespace provides mathematical constants as template variables with C++17-C++26 optimizations.

### Usage

```cpp
#include <microla/constants.hpp>

using namespace microla;

// Use with specific type
float pi_f = constants::pi<float>;
double pi_d = constants::pi<double>;

// Default is double
auto default_pi = constants::pi<>;  // double

// In expressions
float angle = constants::half_pi<float>;
float degrees = radians * constants::rad_to_deg<float>;
```

### Available Constants

**Fundamental Mathematical Constants:**
- `pi<T>` - Pi (3.14159...)
- `two_pi<T>` - 2π (6.28318...)
- `half_pi<T>` - π/2 (1.57079...)
- `quarter_pi<T>` - π/4 (0.78539...)
- `e<T>` - Euler's number (2.71828...)
- `golden_ratio<T>` - φ = (1 + √5) / 2 (1.61803...)
- `sqrt2<T>` - √2 (1.41421...)
- `sqrt3<T>` - √3 (1.73205...)
- `ln2<T>` - Natural log of 2 (0.69314...)
- `ln10<T>` - Natural log of 10 (2.30258...)

**Conversion Factors:**
- `deg_to_rad<T>` - Multiply degrees by this to get radians (π/180)
- `rad_to_deg<T>` - Multiply radians by this to get degrees (180/π)

**Epsilon Values:**
- `epsilon<T>` - Generic epsilon for floating-point comparisons (1e-6)
- `epsilon_f` - Single precision epsilon (1e-6f)
- `epsilon_d` - Double precision epsilon (1e-12)

**Physical Constants (Optional):**
- `gravity<T>` - Earth's gravity (9.80665 m/s²)
- `speed_of_light<T>` - Speed of light in vacuum (299,792,458 m/s)

### Features

- **Type-safe**: Template variables ensure correct type usage
- **Compile-time**: All constants are `constexpr` and can be used in compile-time expressions
- **C++17 optimization**: Uses `inline` variables when available to avoid ODR violations
- **Independent**: Can be used without other library components
- **High precision**: 20 decimal places for maximum accuracy

---

## Vec Class

### Description

`Vec<T, N>` represents an N-dimensional vector with components of type T.

**Template Parameters:**
- `T`: Component type (typically `float` or `double`)
- `N`: Dimension (number of components)

**Memory Layout:**
- Contiguous storage: `alignas(16) T data[N]`
- Common indices: `X=0, Y=1, Z=2, W=3`

### Query Methods

```cpp
static constexpr std::size_t size() noexcept;
```

Returns the number of elements in the vector (N). This is a compile-time constant.

**Example:**

```cpp
Vec<float, 3> v;
std::cout << "Vec3 has " << v.size() << " elements\n";  // Output: 3

// Can be used in constexpr contexts
static_assert(Vec<float, 3>::size() == 3, "Vec3 has 3 elements");

// Generic programming
template<typename T, std::size_t N>
void process_vec(const Vec<T, N>& v) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        // Process each element
    }
}
```

### Construction

```cpp
// Default constructor (uninitialized)
Vec<float, 3> v1;

// From individual components
Vec<float, 3> v2(1.0f, 2.0f, 3.0f);

// From array
float arr[] = {1.0f, 2.0f, 3.0f};
Vec<float, 3> v3(arr);

// Copy constructor
Vec<float, 3> v4 = v2;

// Static factory methods
Vec<float, 3> zero = Vec<float, 3>::zero();       // (0, 0, 0)
Vec<float, 3> one = Vec<float, 3>::one();         // (1, 1, 1)
Vec<float, 3> unit_x = Vec<float, 3>::unit_x();   // (1, 0, 0)
Vec<float, 3> unit_y = Vec<float, 3>::unit_y();   // (0, 1, 0)
Vec<float, 3> unit_z = Vec<float, 3>::unit_z();   // (0, 0, 1)
```

### Component Access

```cpp
Vec<float, 3> v(1.0f, 2.0f, 3.0f);

// Subscript operator
float x = v[0];              // 1.0
float y = v[Vec<float,3>::Y]; // 2.0

// Named accessors (for N >= 1,2,3,4)
float x = v.x();  // 1.0
float y = v.y();  // 2.0
float z = v.z();  // 3.0
```

### Arithmetic Operations

```cpp
Vec<float, 3> a(1.0f, 2.0f, 3.0f);
Vec<float, 3> b(4.0f, 5.0f, 6.0f);

// Addition
Vec<float, 3> sum = a + b;           // (5, 7, 9)
a += b;                              // In-place addition

// Subtraction
Vec<float, 3> diff = a - b;          // (-3, -3, -3)
a -= b;

// Scalar multiplication
Vec<float, 3> scaled = a * 2.0f;     // (2, 4, 6)
scaled = 2.0f * a;                   // Commutative
a *= 2.0f;

// Scalar division
Vec<float, 3> divided = a / 2.0f;    // (0.5, 1, 1.5)
a /= 2.0f;

// Negation
Vec<float, 3> neg = -a;              // (-1, -2, -3)
```

### Vector Operations

```cpp
Vec<float, 3> a(1.0f, 0.0f, 0.0f);
Vec<float, 3> b(0.0f, 1.0f, 0.0f);

// Dot product
float dot = a.dot(b);                // 0.0
float dot_alt = a | b;               // Operator form

// Cross product (3D only)
Vec<float, 3> cross = a.cross(b);    // (0, 0, 1)
cross = a ^ b;                       // Operator form

// Length
float len = a.length();              // 1.0

// Normalization
Vec<float, 3> unit = a.normalized(); // Unit vector
```

### Advanced Operations

```cpp
Vec<float, 3> a(3.0f, 4.0f, 0.0f);
Vec<float, 3> b(1.0f, 0.0f, 0.0f);

// Projection
Vec<float, 3> proj = a.project(b);   // Component of a along b
// Result: (3, 0, 0)

// Rejection
Vec<float, 3> rej = a.reject(b);     // Component perpendicular to b
// Result: (0, 4, 0)

// Angle between vectors
float angle = a.angle(b);            // In radians

// Signed angle (3D, with normal)
Vec<float, 3> normal(0.0f, 0.0f, 1.0f);
float signed_angle = a.signed_angle(b, normal);

// Rotation (3D)
Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
float rotation_angle = 1.57f;        // 90 degrees
Vec<float, 3> rotated = a.rotate(axis, rotation_angle);
```

### Utility Functions

```cpp
Vec<float, 3> a(1.0f, -2.0f, 3.0f);
Vec<float, 3> b(0.5f, 1.5f, 2.0f);

// Element-wise operations
Vec<float, 3> hadamard = a.hadamard(b);  // (0.5, -3, 6)
Vec<float, 3> min_vec = a.min(b);        // (0.5, -2, 2)
Vec<float, 3> max_vec = a.max(b);        // (1, 1.5, 3)

// Min/Max elements
float min_elem = a.min_element();        // -2.0
float max_elem = a.max_element();        // 3.0

// Interpolation
Vec<float, 3> interp = a.lerp(b, 0.5f);              // Linear interpolation
Vec<float, 3> cubic = a.cubic_hermite(b, t1, t2, 0.5f);  // Cubic Hermite spline
Vec<float, 3> catmull = a.catmull_rom(b, p0, p2, 0.5f);  // Catmull-Rom spline

// Safe normalization (returns zero if length < epsilon)
Vec<float, 3> safe = a.safe_normalized(1e-6f);

// Clamping
Vec<float, 3> clamped = a.clamped(-1.0f, 2.0f);  // Clamp each component to range
Vec<float, 3> saturated = a.saturated();         // Clamp to [0, 1]

// Swizzling (component reordering)
Vec<float, 2> xy = a.xy();                // (1, -2)
Vec<float, 3> xyz = a.xyz();              // (1, -2, 3)
Vec<float, 2> xz = a.xz();                // (1, 3)
Vec<float, 2> yz = a.yz();                // (-2, 3)

// Homogeneous coordinates
Vec<float, 4> v4 = a.to_homogeneous();    // (1, -2, 3, 1)
Vec<float, 3> v3 = v4.from_homogeneous(); // Perspective division by w

// Comparison
bool equal = a == b;                     // Exact equality
bool approx = a.approx_equal(b, 1e-6f);  // Approximate equality
```

---

## VectorView Class

### Description

`VectorView<T, N>` represents a non-owning view into an N-element vector stored elsewhere. It provides element access and small utility operations without owning or allocating storage.

**Template Parameters:**
- `T`: Component type (e.g. `float` or `double`)
- `N`: Number of elements in the view

**Characteristics:**
- Non-owning: the referenced data must outlive the view.
- Supports a `stride` to view strided/subsampled data.
- Debug bounds-checking is enabled when `MICROLA_DEBUG` is defined.

### Construction

```cpp
// View into an existing C-style array (parent_size=6, start=1, stride=2)
float data[] = {0,1,2,3,4,5};
VectorView<float, 3> v(data, /*parent_size=*/6, /*start_index=*/1, /*stride=*/2); // views {1,3,5}

// From an owning Vec: use the Vec's data pointer
Vec<float,4> base{1,2,3,4};
VectorView<float,4> whole(base.data(), /*parent_size=*/4, /*start_index=*/0);
```

### Element Access

```cpp
float x = v[0];   // read
v[1] = 7.0f;      // write (mutates underlying storage)
```

### Operations

```cpp
// Return an owning copy of the view contents
Vec<float, 3> copied = v.toVec();

// Copy elements from a Vec into the view (writes into parent storage)
v.set(copied);

// Fill the underlying elements referenced by the view
v.fill(0.0f);
```

### Query Methods

```cpp
constexpr std::size_t size() noexcept;   // Number of elements (N)
constexpr std::size_t stride() noexcept; // Stride between elements in parent storage
```

### Safety

- The view does not manage lifetime of the underlying storage. Use with care.
- Bounds checks are only present when `MICROLA_DEBUG` is enabled; otherwise access is unchecked for performance.

---

## Mat Class

### Description

`Mat<T, R, C>` represents an R×C matrix with components of type T.

**Template Parameters:**
- `T`: Component type (typically `float` or `double`)
- `R`: Number of rows
- `C`: Number of columns

**Memory Layout:**
- Row-major storage: `alignas(16) T data[R * C]`
- Element at (i,j): `data[(i * C) + j]`

### Query Methods

```cpp
static constexpr std::size_t size() noexcept;
static constexpr std::size_t rows() noexcept;
static constexpr std::size_t cols() noexcept;
```

Returns the dimensions of the matrix as compile-time constants.
- `size()`: Total number of elements (R × C)
- `rows()`: Number of rows (R)
- `cols()`: Number of columns (C)

**Example:**

```cpp
Mat<float, 3, 4> m;
std::cout << "Matrix is " << m.rows() << "x" << m.cols()
          << " (" << m.size() << " elements)\n";
// Output: Matrix is 3x4 (12 elements)

// Can be used in constexpr contexts
static_assert(Mat<float, 3, 4>::rows() == 3, "3 rows");
static_assert(Mat<float, 3, 4>::cols() == 4, "4 columns");
static_assert(Mat<float, 3, 4>::size() == 12, "12 total elements");

// Generic programming
template<typename T, std::size_t R, std::size_t C>
void process_matrix(const Mat<T, R, C>& m) {
    std::cout << "Processing " << m.rows() << "x" << m.cols() << " matrix\n";
}
```

### Construction

```cpp
// Default constructor (uninitialized)
Mat<float, 3, 3> m1;

// From array
float arr[] = {1, 0, 0,
               0, 1, 0,
               0, 0, 1};
Mat<float, 3, 3> m2(arr);

// Static constructors
Mat<float, 3, 3> zeros = Mat<float, 3, 3>::zero();
Mat<float, 3, 3> ones = Mat<float, 3, 3>::ones();
Mat<float, 3, 3> identity = Mat<float, 3, 3>::identity();
```

### Element Access

```cpp
Mat<float, 3, 3> m = Mat<float, 3, 3>::identity();

// Double subscript (row, col)
float element = m[0][0];     // 1.0
m[1][2] = 5.0f;

// at() method
float val = m.at(0, 0);      // 1.0

// Row and column extraction
Vec<float, 3> row0 = m.row(0);
Vec<float, 3> col1 = m.col(1);
```

### Arithmetic Operations

```cpp
Mat<float, 2, 2> a, b;
// ... initialize a and b

// Addition/Subtraction
Mat<float, 2, 2> sum = a + b;
Mat<float, 2, 2> diff = a - b;
a += b;
a -= b;

// Scalar operations
Mat<float, 2, 2> scaled = a * 2.0f;
scaled = 2.0f * a;           // Commutative
a *= 2.0f;
a /= 2.0f;

// Negation
Mat<float, 2, 2> neg = -a;
```

### Matrix Operations

```cpp
Mat<float, 3, 3> A;
Mat<float, 3, 4> B;
Vec<float, 3> v;

// Matrix multiplication
Mat<float, 3, 4> C = A * B;
C = mul(A, B);               // Free function form

// Matrix-vector multiplication
Vec<float, 3> result = A * v;
result = mul(A, v);

// Transpose
Mat<float, 3, 3> AT = A.transpose();

// Frobenius norm
float norm = A.frobenius_norm();
Mat<float, 3, 3> normalized = A.normalized();
```

### Utility Functions

```cpp
Mat<float, 3, 3> A, B;

// Element-wise operations
Mat<float, 3, 3> hadamard = A.hadamard(B);

// Min/Max
float min_elem = A.min_element();
float max_elem = A.max_element();

// Fill
A.fill(0.0f);

// Interpolation
Mat<float, 3, 3> interp = A.lerp(B, 0.5f);

// Comparison
bool equal = A == B;
bool approx = A.approx_equal(B, 1e-6f);

// Swap
A.swap(B);
```

---

## SquareMat Class

`SquareMat<T, N>` extends `Mat<T, N, N>` with square matrix operations.

### Additional Operations

```cpp
SquareMat<float, 3> A = SquareMat<float, 3>::identity();

// Trace (sum of diagonal)
float tr = A.trace();

// Determinant
float det = A.determinant();

// Inverse
SquareMat<float, 3> inv = A.inverse();

// Safe inverse with condition number checking (variant-based)
auto checked_result = A.inverse_checked(1e6f);
if (std::holds_alternative<SquareMat<float, 3>>(checked_result)) {
    SquareMat<float, 3> checked_inv = std::get<SquareMat<float, 3>>(checked_result);
} else {
    // std::get<Mat<float,3,3>::InverseError>(checked_result)
}

// Rank
std::size_t rank = A.rank();
```

### 3D Rotation Matrices

```cpp
// Runtime rotation around coordinate axes (arbitrary angles)
SquareMat<float, 3> Rx = SquareMat<float, 3>::rotation_x(1.57f); // 90° around X
SquareMat<float, 3> Ry = SquareMat<float, 3>::rotation_y(1.57f); // 90° around Y
SquareMat<float, 3> Rz = SquareMat<float, 3>::rotation_z(1.57f); // 90° around Z

// Compile-time rotation for special angles (0°, 90°, 180°, 270°)
// These are constexpr and can be evaluated at compile time
constexpr auto R90x = SquareMat<float, 3>::rotation_x_deg<90>();   // 90° around X
constexpr auto R180y = SquareMat<float, 3>::rotation_y_deg<180>(); // 180° around Y
constexpr auto R270z = SquareMat<float, 3>::rotation_z_deg<270>(); // 270° around Z

// 2D rotation (compile-time for special angles)
constexpr auto R2D = SquareMat<float, 2>::rotation_deg<90>();      // 90° in 2D

// Example: Use in constexpr context
constexpr SquareMat<float, 3> transform = SquareMat<float, 3>::rotation_z_deg<90>();
// This matrix is computed at compile time, zero runtime cost!
```

// Rotation from axis and angle
Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
SquareMat<float, 3> R = SquareMat<float, 3>::rotation_axis_angle(axis, 1.57f);

// Rotation from one vector to another
Vec<float, 3> from(1.0f, 0.0f, 0.0f);
Vec<float, 3> to(0.0f, 1.0f, 0.0f);
SquareMat<float, 3> R2 = SquareMat<float, 3>::rotation_from_to(from, to);

// Look-at matrix
Vec<float, 3> direction(1.0f, 0.0f, 0.0f);
Vec<float, 3> up(0.0f, 0.0f, 1.0f);
SquareMat<float, 3> lookAt = SquareMat<float, 3>::look_at(direction, up);

// Extract Euler angles
Vec<float, 3> euler = R.euler_angles();  // (roll, pitch, yaw)

```

### Scale and Translation

```cpp

// Scale matrix
Vec<float, 3> scale_factors(2.0f, 3.0f, 4.0f);
SquareMat<float, 3> S1 = SquareMat<float, 3>::scale(scale_factors);
SquareMat<float, 3> S2 = SquareMat<float, 3>::scale(2.0f); // Uniform scale

// Translation (4x4 only)
Vec<float, 3> translation(10.0f, 20.0f, 30.0f);
SquareMat<float, 4> T = SquareMat<float, 4>::translation(translation);

```

---

## Quaternion Class

### Description

`Quaternion<T>` represents a quaternion for 3D rotations with optimized memory layout.

**Memory Layout:**
- `alignas(16) T data[4]` stored as `[x, y, z, w]`
- Imaginary part (x,y,z) at indices 0-2 enables `Vec<T,3>` optimizations
- Index constants: `X=0, Y=1, Z=2, W=3`

### Construction

```cpp

// Default constructor (identity: w=1, xyz=0)
Quaternion<float> q1;

// From components (w, x, y, z)
Quaternion<float> q2(1.0f, 0.0f, 0.0f, 0.0f);

// From axis and angle
Vec<float, 3> axis(0.0f, 0.0f, 1.0f);  // Z-axis
float angle = 1.57f;                    // 90 degrees
Quaternion<float> q3(axis, angle);

// From rotation matrix
Mat<float, 3, 3> R = SquareMat<float, 3>::rotation_z(1.57f);
Quaternion<float> q4(R);

// From array [x, y, z, w]
float arr[] = {0.0f, 0.0f, 0.7071f, 0.7071f};
Quaternion<float> q5(arr);

// Identity quaternion
Quaternion<float> identity = Quaternion<float>::identity();

```

### Component Access

```cpp

Quaternion<float> q(0.7071f, 0.0f, 0.0f, 0.7071f); // 90° around X

// Subscript operator
float w = q[Quaternion<float>::W];  // 0.7071
float x = q[Quaternion<float>::X];  // 0.7071

// Named accessors
float w_val = q.w();
float x_val = q.x();
float y_val = q.y();
float z_val = q.z();

// Imaginary part as Vec<T,3>
Vec<float, 3> imag = q.vec();       // Returns copy of (x,y,z)

```

### Arithmetic Operations

```cpp

Quaternion<float> q1(1.0f, 0.1f, 0.2f, 0.3f);
Quaternion<float> q2(1.0f, 0.4f, 0.5f, 0.6f);

// Addition/Subtraction
Quaternion<float> sum = q1 + q2;
q1 += q2;
Quaternion<float> diff = q1 - q2;
q1 -= q2;

// Scalar multiplication/division
Quaternion<float> scaled = q1 * 2.0f;
scaled = 2.0f * q1;              // Commutative
q1 *= 2.0f;
q1 /= 2.0f;

// Quaternion multiplication (rotation composition)
Quaternion<float> product = q1 * q2;
q1 *= q2;

// Negation
Quaternion<float> neg = -q1;

```

### Quaternion Operations

```cpp

Quaternion<float> q(axis, angle);

// Conjugate (inverse rotation for unit quaternions)
Quaternion<float> conj = q.conjugate();

// Norm (magnitude)
float norm = q.norm();

// Normalize
Quaternion<float> unit = q.normalized();

// Inverse
Quaternion<float> inv = q.inverse();

// Dot product
float dot = q1.dot(q2);

// Convert to rotation matrix
Mat<float, 3, 3> R = q.to_matrix();

```

### Rotating Vectors

```cpp

Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
Quaternion<float> q(axis, 1.57f);      // 90° around Z

Vec<float, 3> point(1.0f, 0.0f, 0.0f);
Vec<float, 3> rotated = q.rotate(point);
// Result: (0, 1, 0) - point rotated 90° around Z

```

### Euler Angle Conversions

```cpp

// Convert quaternion to Euler angles (roll, pitch, yaw)
Quaternion<float> q = /* ... */;
Vec<float, 3> euler = q.to_euler();    // Returns (roll, pitch, yaw) in radians

// Extract individual angles
float roll = q.roll();                  // Rotation about X axis
float pitch = q.pitch();                // Rotation about Y axis
float yaw = q.yaw();                    // Rotation about Z axis

// Create quaternion from Euler angles
Vec<float, 3> euler(0.1f, 0.2f, 0.3f);  // (roll, pitch, yaw)
Quaternion<float> q1 = Quaternion<float>::from_euler(euler);
Quaternion<float> q2 = Quaternion<float>::from_euler(0.1f, 0.2f, 0.3f);

```

### Interpolation

```cpp

Quaternion<float> q1 = Quaternion<float>::identity();
Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
Quaternion<float> q2(axis, 3.14f);     // 180° rotation

// Linear interpolation (not normalized)
Quaternion<float> mid = q1.lerp(q2, 0.5f);

// Spherical linear interpolation (smooth rotation)
Quaternion<float> slerp_mid = q1.slerp(q2, 0.5f);
// Result: 90° rotation (halfway between 0° and 180°)

```

### Comparison

```cpp

Quaternion<float> q1, q2;

// Exact equality
bool equal = q1 == q2;
bool not_equal = q1 != q2;

// Approximate equality
bool approx = q1.approx_equal(q2, 1e-6f);

```

---

## Version API

### Description

The version API allows runtime version checking and compatibility verification.

**Header:** `<microla/version.hpp>`

### Version Information

```cpp

#include <microla/version.hpp>

using namespace microla;

// Get version string
std::string version = get_version_string();  // "0.0.1"

// Get numeric version
uint32_t version_num = get_version_number(); // 1

// Check minimum version
if (version_at_least(0, 0, 1)) {
    // Use library features
}

```

### Version Macros

```cpp

// Compile-time version macros
MICROLA_VERSION_MAJOR  // 0
MICROLA_VERSION_MINOR  // 0
MICROLA_VERSION_PATCH  // 1
MICROLA_VERSION_STRING // "0.0.1"
MICROLA_VERSION_NUMBER // 1

```

### Version Struct

```cpp

namespace microla {
    struct Version {
        static constexpr uint8_t major = 0;
        static constexpr uint8_t minor = 0;
        static constexpr uint8_t patch = 1;
    };
}

```

---

## SIMD Optimizations

The library supports hardware acceleration on ARM processors with a three-tier optimization strategy:
**NEON** (Cortex-A/ARM64) → **CMSIS-DSP** (Cortex-M) → **Generic C++**

### ARM NEON Optimizations

**Platforms:** ARM Cortex-A, ARM64/AArch64, Apple Silicon (M1/M2/M3)

#### Enabling NEON

```cpp

// In your build configuration:

#define CONFIG_MICROLA_NEON

// CMake:
cmake -DMICROLA_ENABLE_NEON=ON ..

// Then include the library

#include <microla/microla.hpp>

#include <microla/quaternion.hpp>

```

#### NEON-Optimized Operations

When `CONFIG_MICROLA_NEON` is defined and `T=float`, the following use ARM NEON intrinsics:

**Vec<float, 2> Operations:**
- Addition: `vadd_f32` (64-bit NEON)
- Subtraction: `vsub_f32`
- Scalar multiplication: `vmul_f32` with `vdup_n_f32`
- Dot product: `vmul_f32` + `vpadd_f32` horizontal reduction

**Vec<float, 3> Operations:**
- Addition: `vaddq_f32` (pads to 4 elements for 128-bit alignment)
- Dot product: `vmulq_f32` + horizontal reduction

**Vec<float, 4> Operations:**
- Addition: `vaddq_f32` (128-bit NEON)
- Subtraction: `vsubq_f32`
- Scalar multiplication: `vmulq_f32` with `vdupq_n_f32`
- Dot product: `vmulq_f32` + horizontal reduction

**Quaternion<float> Operations:**
- Addition: `vaddq_f32` (4 components fit perfectly in 128-bit register)
- Subtraction: `vsubq_f32`
- Negation: `vnegq_f32`
- Scalar multiplication: `vmulq_f32`
- Dot product: `vmulq_f32` + horizontal reduction
- Norm: NEON dot product + `std::sqrt`

### CMSIS-DSP Optimizations

**Platforms:** ARM Cortex-M processors (M0/M0+/M3/M4/M7/M33/M55/M85)

#### Enabling CMSIS

```cpp

// In your build configuration:

#define CONFIG_MICROLA_CMSIS

// Then include the library

#include <microla/microla.hpp>

#include <microla/quaternion.hpp>

```

#### CMSIS-Optimized Operations

When `CONFIG_MICROLA_CMSIS` is defined and `T=float`, the following operations use CMSIS-DSP:

**Vec Operations:**
- Addition: `arm_add_f32`
- Subtraction: `arm_sub_f32`
- Scalar multiplication: `arm_scale_f32`
- Dot product: `arm_dot_prod_f32`

**Mat Operations:**
- Matrix multiplication: `arm_mat_mult_f32`
- Matrix-vector multiplication: `arm_mat_vec_mult_f32`
- Transpose: `arm_mat_trans_f32`
- Inverse: `arm_mat_inverse_f32`

**Quaternion Operations:**
- Addition: `arm_add_f32` (4 elements)
- Subtraction: `arm_sub_f32` (4 elements)
- Scalar multiplication: `arm_scale_f32` (4 elements)
- Negation: `arm_negate_f32` (4 elements)
- Dot product: `arm_dot_prod_f32` (4 elements)
- Quaternion multiplication: `arm_quaternion_product_f32`

### Performance Notes

- **Three-tier strategy**: NEON preferred on Cortex-A/ARM64, CMSIS-DSP on Cortex-M, generic C++ fallback
- **NEON operations**: Only used for `float` types (not `double`)
- **CMSIS operations**: Only used for `float` types (not `double`)
- **Zero overhead**: Optimizations are compile-time conditional, no runtime checks
- **Vec<T,3> alignment**: NEON uses padding to 4 elements for optimal 128-bit register usage
- **Quaternion efficiency**: 4-component structure maps perfectly to 128-bit NEON registers
- **Performance gains**: Typically 2-4x speedup on supported platforms

### Namespace Usage

All library components are in the `microla` namespace:

```cpp

#include <microla/microla.hpp>

#include <microla/quaternion.hpp>

#include <microla/version.hpp>

// Option 1: Explicit namespace
microla::Vec<float, 3> v;
microla::Quaternion<float> q;

// Option 2: Using namespace (recommended)
using namespace microla;
Vec<float, 3> v;
Quaternion<float> q;

```

---

## Mathematical Utilities

### Constants

The `microla::constants` namespace provides common mathematical constants:

```cpp

#include <microla/microla.hpp>

using namespace microla;

// Mathematical constants (template parameter defaults to double)
float pi = constants::pi<float>;               // 3.14159...
float two_pi = constants::two_pi<float>;       // 2π
float half_pi = constants::half_pi<float>;     // π/2
float quarter_pi = constants::quarter_pi<float>; // π/4

// Angle conversion multipliers
float deg_mult = constants::deg_to_rad<float>; // 0.01745...
float rad_mult = constants::rad_to_deg<float>; // 57.2958...

// Default epsilon values
float eps = constants::epsilon<float>;         // 1e-6
float eps_f = constants::epsilon_f;            // 1e-6f
double eps_d = constants::epsilon_d;           // 1e-12

```

### Angle Utilities

```cpp

// Convert between degrees and radians
float rad = deg_to_rad(90.0f);                 // 1.5708 (π/2)
float deg = rad_to_deg(3.14159f);              // 180.0

// Wrap angles to standard ranges
float wrapped = wrap_pi(3.5f);                 // Wrap to [-π, π]
float wrapped2 = wrap_two_pi(7.0f);            // Wrap to [0, 2π]

// Calculate shortest angular distance
float distance = angle_distance(0.1f, 6.2f);   // Shortest path from angle1 to angle2

```

### Clamping Utilities

```cpp

// Clamp value to range
float val = clamp(1.5f, 0.0f, 1.0f);           // Returns 1.0
int ival = clamp(-5, 0, 100);                  // Returns 0

// Saturate to [0, 1]
float sat = saturate(1.5f);                    // Returns 1.0
float sat2 = saturate(-0.5f);                  // Returns 0.0

```

---

## Common Patterns

### 3D Rotation Pipeline

```cpp

using namespace microla;

// Define rotation axis and angle
Vec<float, 3> axis(0.0f, 1.0f, 0.0f);  // Y-axis
float angle = 0.785f;                   // 45 degrees

// Create quaternion
Quaternion<float> q(axis, angle);

// Rotate multiple points
Vec<float, 3> point1(1.0f, 0.0f, 0.0f);
Vec<float, 3> point2(0.0f, 0.0f, 1.0f);

Vec<float, 3> rotated1 = q.rotate(point1);
Vec<float, 3> rotated2 = q.rotate(point2);

// Compose rotations
Quaternion<float> q2(Vec<float, 3>(1.0f, 0.0f, 0.0f), 0.5f);
Quaternion<float> combined = q * q2;  // Apply q2, then q

```

### Accelerometer Angle Calculation

```cpp

using namespace microla;

Vec<float, 3> accel1(0.0f, 0.0f, 1.0f);  // First accelerometer
Vec<float, 3> accel2(0.707f, 0.0f, 0.707f);  // Second accelerometer

// Angle between accelerometers
float angle = accel1.angle(accel2);  // Unsigned angle

// Signed angle relative to a normal
Vec<float, 3> normal(0.0f, 1.0f, 0.0f);
float signed_angle = accel1.signed_angle(accel2, normal);

// Create rotation to align accel1 to accel2
SquareMat<float, 3> R = SquareMat<float, 3>::rotation_from_to(accel1, accel2);
Quaternion<float> q(R);

```

### Transformation Chain

```cpp

using namespace microla;

// Build transformation matrix (4x4 for 3D + translation)
Vec<float, 3> translation(10.0f, 0.0f, 0.0f);
Vec<float, 3> scale(2.0f, 2.0f, 2.0f);

SquareMat<float, 4> T = SquareMat<float, 4>::translation(translation);
SquareMat<float, 3> S = SquareMat<float, 3>::scale(scale);
SquareMat<float, 3> R = SquareMat<float, 3>::rotation_z(1.57f);

// Combine 3x3 operations
SquareMat<float, 3> transform = R * S;  // Rotate then scale

```

### Linear Interpolation Animation

```cpp

using namespace microla;

// Smooth rotation animation
Quaternion<float> start = Quaternion<float>::identity();
Quaternion<float> end(Vec<float, 3>(0, 0, 1), 3.14f);

// Animate over 100 steps
for (int i = 0; i <= 100; ++i) {
    float t = i / 100.0f;
    Quaternion<float> current = start.slerp(end, t);

    Vec<float, 3> point(1.0f, 0.0f, 0.0f);
    Vec<float, 3> animated = current.rotate(point);
    // Use animated position...
}

```

### Working with POD Types

```cpp

using namespace microla;

// Direct memory access for DMA transfers
Vec<float, 3> data;
float* raw_ptr = data.ptr();
// Send raw_ptr to hardware...

// Size information
std::size_t byte_size = sizeof(data);        // 16 bytes (aligned)
std::size_t element_count = data.size();     // Not available for Vec
// For Vec, size is template parameter N

// Matrix data access
Mat<float, 3, 3> matrix;
float* mat_ptr = matrix.ptr();
std::size_t mat_elements = matrix.size();    // 9

```

---

## Type Aliases

Common type aliases for convenience (in `microla` namespace):

```cpp

namespace microla {
    using Vec2f = Vec<float, 2>;
    using Vec3f = Vec<float, 3>;
    using Vec4f = Vec<float, 4>;

    using Mat2f = Mat<float, 2, 2>;
    using Mat3f = Mat<float, 3, 3>;
    using Mat4f = Mat<float, 4, 4>;

    using Quatf = Quaternion<float>;

    // Double precision variants
    using Vec3d = Vec<double, 3>;
    using Mat3d = Mat<double, 3, 3>;
    using Quatd = Quaternion<double>;
}

```

## Build Requirements

- **C++ Standard**: C++17 or later
- **Dependencies**: None (ARM NEON/CMSIS-DSP optional)
- **Compiler Support**: GCC, Clang, MSVC, ARM Compiler 6, IAR Embedded Workbench for ARM
- **SIMD Support**: ARM NEON (Cortex-A/ARM64), CMSIS-DSP (Cortex-M)
- **IDE Support**: Visual Studio, CLion, Eclipse, IAR Embedded Workbench, VS Code

For IAR-specific integration instructions, see [IAR Integration Guide](IAR_Integration.md).

## Version

Current Version: **0.0.1**
Check runtime version: `microla::get_version_string()`

## License

Apache License 2.0

---

**Date**: January 31, 2026

## Embedded-safe APIs

This library provides non-throwing, allocation-free APIs intended for constrained embedded builds.

- `bool Mat<T,R,C>::inverse(Mat<T,R,C>& out) const noexcept` — non-throwing inversion; returns `false` if the matrix is singular. Preferred for builds with `MICROLA_NO_EXCEPTIONS` enabled.
- `std::optional<Mat<T,R,C>> Mat<T,R,C>::try_inverse() const noexcept` — non-throwing inversion with optional return.
- `std::variant<Mat<T,R,C>, Mat<T,R,C>::InverseError> Mat<T,R,C>::inverse_checked(T threshold = T(1e6)) const noexcept` — inversion with condition number checking and typed error reporting (`Singular`, `IllConditioned`).
- `std::optional<Mat<T,R,C>> Mat<T,R,C>::cholesky() const noexcept` — Cholesky decomposition for SPD matrices; returns `std::nullopt` when the matrix is not positive-definite.
- `bool Mat<T,R,C>::eigenvalues_qr(T* out_buffer, std::size_t out_size, ...) const noexcept` — writes eigenvalues into a caller-provided buffer and returns `true` on success. Use this to avoid heap allocations in embedded environments.
- `std::variant<std::vector<T>, Mat<T,R,C>::EigenError> Mat<T,R,C>::eigenvalues_qr(...) const` — convenience wrapper with typed convergence errors.

Notes:
- The convenience wrapper `Mat inverse() const` continues to exist for host builds; prefer optional/variant or buffer-based APIs for embedded/no-exceptions/no-heap configurations.
- To build for embedded targets, define `-DMICROLA_NO_EXCEPTIONS -DMICROLA_NO_DYNAMIC_ALLOC -DMICROLA_EMBEDDED` (see `QUICKSTART.md` for commands). A GitHub Actions workflow exists to validate these settings automatically: `.github/workflows/embedded-build.yml`.
