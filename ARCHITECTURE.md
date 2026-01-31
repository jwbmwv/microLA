# MicroLA Architecture

This document describes the design decisions, architecture patterns, and implementation strategies used in MicroLA.

## Table of Contents

1. [Design Philosophy](#design-philosophy)
2. [Architecture Overview](#architecture-overview)
3. [Zero-Overhead Abstraction Strategy](#zero-overhead-abstraction-strategy)
4. [Header-Only Design](#header-only-design)
5. [Template Metaprogramming](#template-metaprogramming)
6. [SIMD Optimization Architecture](#simd-optimization-architecture)
7. [Platform Abstraction](#platform-abstraction)
8. [Type System](#type-system)
9. [Memory Layout](#memory-layout)
10. [Compile-Time Computation](#compile-time-computation)
11. [Safety Architecture](#safety-architecture)
12. [Performance Considerations](#performance-considerations)
13. [Future Architecture Direction](#future-architecture-direction)

---

## Design Philosophy

MicroLA is built on several core principles:

### 1. **Zero-Overhead Abstractions**

No runtime cost for abstractions. Compiled code should be identical to hand-written optimal C.

```cpp
// High-level code
Vec3f result = a.cross(b);

// Compiles to same assembly as:
float result_x = a.y * b.z - a.z * b.y;
float result_y = a.z * b.x - a.x * b.z;
float result_z = a.x * b.y - a.y * b.x;
```

### 2. **Embedded-First**

Designed for resource-constrained systems:
- No dynamic allocation
- Stack-based storage
- Predictable performance
- Minimal code size
- C++17 baseline

### 3. **Type Safety Without Runtime Cost**

Leverage compile-time type checking:
- Dimension mismatches caught at compile time
- Type-safe matrix operations
- No runtime type checking overhead

### 4. **Explicit Over Implicit**

Clear intent in API design:
- Explicit conversions
- Named constructors
- Clear function names
- Minimal operator overloading surprises

---

## Architecture Overview

### Component Hierarchy

```
microla.hpp (main header)
├── compiler_features.hpp    # C++ standard detection
├── constants.hpp            # Mathematical constants
├── simd_helpers.hpp         # Platform-agnostic SIMD wrappers
├── vector.hpp               # Generic Vec<T,N>
├── matrix.hpp               # Generic Mat<T,R,C> and SquareMat<T,N>
├── quaternion.hpp           # Quaternion<T>
├── matrix_view.hpp          # Non-owning matrix views
├── vector_view.hpp          # Non-owning vector views with stride
├── kalman.hpp               # Standard Kalman filter
├── extended_kalman.hpp      # Extended Kalman filter
├── safe_math.hpp            # Safety-critical operations
├── fast_math.hpp            # Fast approximations
├── numerical_stability.hpp  # Kahan summation, stable hypot
├── high_precision.hpp       # Error-free transformations
└── resource_checks.hpp      # Compile-time size/layout checks

Note: geometry.hpp and version.hpp are NOT
included by microla.hpp — include them separately.
```

### Dependency Graph

```
compiler_features.hpp
    ↓
constants.hpp, simd_helpers.hpp, high_precision.hpp
    ↓
vector.hpp (depends on constants, simd_helpers)
    ↓
matrix.hpp (depends on vector, constants, simd_helpers)
    ↓
quaternion.hpp (depends on vector, matrix, constants)
    ↓
geometry.hpp (depends on vector, matrix, high_precision)
numerical_stability.hpp (depends on compiler_features, high_precision)
    ↓
microla.hpp (convenience header — includes all except geometry.hpp and version.hpp)
```

**Design Principle:** Minimal dependencies enable:
- Fast compilation
- Selective inclusion
- Clear module boundaries

---

## Zero-Overhead Abstraction Strategy

### Stack Allocation

All types use fixed-size stack arrays:

```cpp
template<typename T, size_t N>
class Vec
{
private:
    T data[N];  // Stack array, no heap allocation
};
```

**Benefits:**
- No allocation overhead
- Cache-friendly
- Deterministic performance
- Suitable for real-time systems

### Inlining Strategy

All functions marked inline or constexpr:

```cpp
// Always inlined in release builds
MICROLA_CONSTEXPR Vec3f cross(const Vec3f& other) const
{
    return Vec3f(
        y() * other.z() - z() * other.y(),
        z() * other.x() - x() * other.z(),
        x() * other.y() - y() * other.x()
    );
}
```

**Compiler optimization levels:**
- `-O2`: Most operations inlined
- `-O3`: All operations inlined, vectorized
- `-Ofast`: Aggressive optimizations, fast math

### Return Value Optimization (RVO)

Rely on compiler RVO/NRVO:

```cpp
// No copies in modern compilers
Mat3f rotateZ(float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);

    return Mat3f(  // RVO eliminates copy
        c, -s, 0,
        s,  c, 0,
        0,  0, 1
    );
}
```

---

## Header-Only Design

### Rationale

**Advantages:**
1. **No build complexity** - Just `#include`
2. **Full optimization** - All code visible to compiler
3. **Template instantiation** - Required for templates
4. **Easy integration** - Copy headers, done

**Disadvantages:**
1. **Compilation time** - All code recompiled per TU
2. **Code bloat** - Templates instantiated in each TU
3. **Debug symbols** - Larger debug builds

### Mitigation Strategies

**Precompiled Headers (PCH):**

```cpp
// Precompile stable headers
#include <microla/microla.hpp>
```

**Explicit Instantiation (extern template):**

```cpp
// In one .cpp file
template class Vec<float, 3>;
template class Mat<float, 4, 4>;

// In headers
extern template class Vec<float, 3>;
extern template class Mat<float, 4, 4>;
```

**Forward Declarations:**

```cpp
// Avoid including full headers
template<typename T, size_t N> class Vec;
template<typename T, size_t R, size_t C> class Mat;
```

---

## Template Metaprogramming

### Compile-Time Dimension Checking

```cpp
template<typename T, size_t R1, size_t C1, size_t C2>
Mat<T, R1, C2> operator*(
    const Mat<T, R1, C1>& lhs,
    const Mat<T, C1, C2>& rhs)
{
    // Dimension mismatch: C1 must match!
    // Compile error if dimensions incompatible
}
```

### SFINAE-Based Specialization

```cpp
// Generic matrix
template<typename T, size_t R, size_t C>
class Mat { /* ... */ };

// Square matrix specialization (R == C)
template<typename T, size_t N>
class SquareMat : public Mat<T, N, N>
{
    // Additional operations: inverse, determinant, trace
};
```

### Template Aliases for Convenience

```cpp
// Type aliases for common dimensions
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;

using Mat3f = SquareMat<float, 3>;
using Mat4f = SquareMat<float, 4>;
```

---

## SIMD Optimization Architecture

### Multi-Level Optimization Strategy

```
┌─────────────────────────────────────┐
│   High-Level API (Vec3f, Mat4f)    │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│  Platform Detection (compile-time)  │
├─────────────────────────────────────┤
│  #if defined(CONFIG_MICROLA_NEON)  │
│  #elif defined(CONFIG_MICROLA_RISCV)│
│  #elif defined(__SSE__)             │
│  #else // Scalar fallback           │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│    SIMD Implementation Layer        │
├─────────────────────────────────────┤
│  • ARM NEON (float32x4_t)          │
│  • ARM MVE (mve_pred16_t)          │
│  • CMSIS-DSP (arm_math.h)          │
│    - arm_mat_mult_f32()            │
│    - arm_dot_prod_f32()            │
│    - arm_mat_add/sub/scale_f32()   │
│  • RISC-V V (vfloat32m1_t)         │
│    - vle32_v_f32m1, vfadd_vv_f32m1 │
│    - vfmul_vv_f32m1, vfredsum...   │
│  • x86 SSE/AVX (future)            │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│      Scalar Fallback Always         │
│         Available                   │
└─────────────────────────────────────┘
```

### CMSIS-DSP Integration

**CMSIS-DSP** is ARM's optimized DSP library for Cortex-M processors, providing hardware-accelerated math functions.

```cpp
// CMSIS-DSP optimized matrix multiply
#if defined(CONFIG_MICROLA_CMSIS) && std::is_same<T, float>::value
    Mat<float, R, OtherC> operator*(const Mat<float, C, OtherC>& other) const
    {
        Mat<float, R, OtherC> result;

        // Initialize CMSIS-DSP matrix instances
        arm_matrix_instance_f32 src1, src2, dst;
        arm_mat_init_f32(&src1, R, C, const_cast<float*>(data));
        arm_mat_init_f32(&src2, C, OtherC, const_cast<float*>(other.data));
        arm_mat_init_f32(&dst, R, OtherC, result.data);

        // Hardware-accelerated multiplication
        arm_mat_mult_f32(&src1, &src2, &dst);

        return result;
    }
#else
    // Scalar fallback...
#endif
```

**CMSIS-DSP Benefits:**
- **Cortex-M4/M7/M33 Optimized**: Leverages DSP instructions (MAC, SIMD)
- **Power Efficient**: Hardware acceleration = lower energy consumption
- **Vendor Tested**: Extensively validated by ARM and silicon vendors
- **Fixed & Float Support**: Q15, Q31 fixed-point + float32 operations

**Typical Performance (Cortex-M7 @ 216 MHz):**
- Matrix 4×4 multiply: **2.9× faster** than scalar
- Vector dot product: **1.6× faster**
- Matrix transpose: **1.9× faster**
- Quaternion normalize: **1.8× faster**

**Integration Pattern:**

```cpp
#ifdef CONFIG_MICROLA_CMSIS
    // Use CMSIS-DSP for float operations
    if (std::is_same<T, float>::value)
    {
        // CMSIS-DSP path...
        return result;
    }
#endif
    // Scalar fallback for non-float types or when CMSIS disabled
```

### SIMD Intrinsics Wrapper Pattern

```cpp
// ARM NEON optimized 4x4 matrix multiply
#if defined(CONFIG_MICROLA_NEON)
    Mat<float, 4, 4> operator*(const Mat<float, 4, 4>& rhs) const
    {
        Mat<float, 4, 4> result;

        // Load matrix columns
        float32x4_t col0 = vld1q_f32(&rhs.data[0]);
        float32x4_t col1 = vld1q_f32(&rhs.data[4]);
        float32x4_t col2 = vld1q_f32(&rhs.data[8]);
        float32x4_t col3 = vld1q_f32(&rhs.data[12]);

        // Process each row
        for (size_t i = 0; i < 4; ++i)
        {
            float32x4_t row = vld1q_f32(&data[i * 4]);

            // Broadcast and multiply-accumulate
            float32x4_t r = vmulq_n_f32(col0, vgetq_lane_f32(row, 0));
            r = vmlaq_n_f32(r, col1, vgetq_lane_f32(row, 1));
            r = vmlaq_n_f32(r, col2, vgetq_lane_f32(row, 2));
            r = vmlaq_n_f32(r, col3, vgetq_lane_f32(row, 3));

            vst1q_f32(&result.data[i * 4], r);
        }

        return result;
    }
#else
    // Scalar fallback - always available
    Mat<float, 4, 4> operator*(const Mat<float, 4, 4>& rhs) const
    {
        Mat<float, 4, 4> result;
        for (size_t i = 0; i < 4; ++i)
        {
            for (size_t j = 0; j < 4; ++j)
            {
                float sum = 0;
                for (size_t k = 0; k < 4; ++k)
                {
                    sum += (*this)(i, k) * rhs(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
#endif
```

### Platform Detection Macros

```cpp
// compiler_features.hpp
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define MICROLA_HAS_NEON 1
#endif

#if defined(__ARM_FEATURE_MVE)
    #define MICROLA_HAS_MVE 1
#endif

#if defined(ARM_MATH_CM4) || defined(ARM_MATH_CM7)
    #define MICROLA_HAS_CMSIS 1
#endif
```

**Configuration:**
- Compile-time (`#define CONFIG_MICROLA_NEON`)
- CMake options (`-DMICROLA_ENABLE_NEON=ON`)
- Kconfig (Zephyr: `CONFIG_MICROLA_NEON=y`)

---

## Platform Abstraction

### Three-Tier Platform Support

#### Tier 1: Desktop/Server (x86_64, ARM64)

- Full C++17/20 support
- SIMD: SSE, AVX, NEON
- Testing: Primary development platform

#### Tier 2: Embedded Linux/RTOS (ARM Cortex-A)

- C++17 minimum
- SIMD: ARM NEON
- Testing: Raspberry Pi, Jetson

#### Tier 3: Microcontrollers (ARM Cortex-M)

- C++17 (IAR, GCC)
- SIMD: CMSIS-DSP, ARM MVE
- Testing: STM32, nRF52, Zephyr QEMU

### Compiler Compatibility

```cpp
// Detect compiler and version
#if defined(__clang__)
    #define MICROLA_COMPILER_CLANG
    #define MICROLA_COMPILER_VERSION __clang_major__
#elif defined(__GNUC__)
    #define MICROLA_COMPILER_GCC
    #define MICROLA_COMPILER_VERSION __GNUC__
#elif defined(_MSC_VER)
    #define MICROLA_COMPILER_MSVC
    #define MICROLA_COMPILER_VERSION _MSC_VER
#elif defined(__ICCARM__)
    #define MICROLA_COMPILER_IAR
    #define MICROLA_COMPILER_VERSION __VER__
#endif

// C++ standard detection
#if __cplusplus >= 202002L
    #define MICROLA_CPP20
#elif __cplusplus >= 201703L
    #define MICROLA_CPP17
#endif
```

### Compatibility Macros

```cpp
// constexpr handling
#define MICROLA_CONSTEXPR constexpr

// if constexpr handling
#define MICROLA_IF_CONSTEXPR if constexpr

// [[nodiscard]] attribute
#define MICROLA_NODISCARD [[nodiscard]]
```

---

## Type System

### Scalar Type Requirements

```cpp
template<typename T>
concept ArithmeticType = requires(T a, T b)
{
    { a + b } -> std::convertible_to<T>;
    { a - b } -> std::convertible_to<T>;
    { a * b } -> std::convertible_to<T>;
    { a / b } -> std::convertible_to<T>;
};
```

**Supported Types:**
- `float` (primary target)
- `double` (high precision)
- `int`, `long`, `short` (integer math)
- Fixed-point types (future)

### Type Aliases

```cpp
// Single precision (default for embedded)
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Mat3f = SquareMat<float, 3>;
using Mat4f = SquareMat<float, 4>;
using Quatf = Quaternion<float>;

// Double precision (scientific computing)
using Vec2d = Vec<double, 2>;
using Vec3d = Vec<double, 3>;
using Mat3d = SquareMat<double, 3>;
using Mat4d = SquareMat<double, 4>;
using Quatd = Quaternion<double>;

// Integer (fixed-point, indices)
using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;
```

---

## Memory Layout

### Row-Major Matrix Storage

```cpp
// Mat<T, R, C> stores in row-major order
Mat<float, 2, 3> m = {
    1, 2, 3,    // Row 0
    4, 5, 6     // Row 1
};

// Memory: [1, 2, 3, 4, 5, 6]
// Access: m(row, col) = data[row * C + col]
```

**Rationale:**
- C/C++ array layout convention
- Cache-friendly row iteration
- Compatible with row vectors
- Natural mathematical notation

### SIMD-Friendly Alignment

```cpp
// 4x4 matrix: 16 floats = 64 bytes
// Naturally aligned for SIMD loads
alignas(16) float data[16];

// ARM NEON: Load 4 floats at once
float32x4_t vec = vld1q_f32(&data[0]);
```

### Structure Layout

```cpp
// Vec<float, 3>: 12 bytes
struct Vec3f
{
    float x, y, z;  // No padding
};

// Mat<float, 4, 4>: 64 bytes
struct Mat4f
{
    float data[16];  // Contiguous
};

// Quaternion<float>: 16 bytes
struct Quatf
{
    float x, y, z, w;  // [x, y, z, w] layout (constructor takes w, x, y, z)
};
```

---

## Compile-Time Computation

### constexpr Functions

```cpp
// Compile-time rotation matrix
MICROLA_CONSTEXPR Mat3f rotateZ(float angle)
{
    // Requires constexpr cos/sin in C++26
    // For now: compile-time for constant angles
    return Mat3f(
        std::cos(angle), -std::sin(angle), 0,
        std::sin(angle),  std::cos(angle), 0,
        0, 0, 1
    );
}

// Use at compile time
constexpr Mat3f rot90 = rotateZ(1.5707963f);  // π/2
```

### Template Metaprogramming

```cpp
// Compile-time factorial
template<size_t N>
struct Factorial
{
    static constexpr size_t value = N * Factorial<N-1>::value;
};

template<>
struct Factorial<0>
{
    static constexpr size_t value = 1;
};

// Compile-time matrix power
template<size_t N>
MICROLA_CONSTEXPR Mat4f matrixPower(const Mat4f& m)
{
    MICROLA_IF_CONSTEXPR (N == 0)
    {
        return Mat4f::identity();
    }
    MICROLA_IF_CONSTEXPR (N == 1)
    {
        return m;
    }
    else
    {
        Mat4f half = matrixPower<N/2>(m);
        Mat4f result = half * half;
        MICROLA_IF_CONSTEXPR (N % 2 == 1)
        {
            result = result * m;
        }
        return result;
    }
}
```

---

## Safety Architecture

### Multi-Level Safety Strategy

```
┌────────────────────────────────┐
│   Compile-Time Safety          │  <-- Type system, dimensions
├────────────────────────────────┤
│   Debug-Time Safety            │  <-- Assertions, bounds checks
├────────────────────────────────┤
│   Runtime Safety (opt-in)      │  <-- Exception handling
├────────────────────────────────┤
│   Sanitizer Support            │  <-- ASan, UBSan, MSan
└────────────────────────────────┘
```

### Compile-Time Safety

```cpp
// Type mismatches caught at compile time
Mat<float, 3, 4> a;
Mat<float, 2, 3> b;
auto c = a * b;  // ❌ Compile error: dimensions don't match
```

### Debug-Time Safety

```cpp
#ifdef MICROLA_DEBUG
    T& at(size_t index)
    {
        assert(index < N && "Index out of bounds");
        return data[index];
    }
#else
    T& at(size_t index)
    {
        return data[index];  // No overhead in release
    }
#endif
```

### Sanitizer Integration

```cpp
// Build with sanitizers
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"

// Catches:
// - Buffer overflows
// - Use-after-free
// - Undefined behavior
// - Memory leaks
```

---

## Performance Considerations

### Optimization Flags

| Flag | Description | Use Case |
|------|-------------|----------|
| `-O2` | Standard optimization | Development |
| `-O3` | Aggressive optimization | Production |
| `-Ofast` | Fast math | Real-time systems |
| `-march=native` | CPU-specific | Maximum performance |
| `-flto` | Link-time optimization | Final binary |

### Profiling Recommendations

```bash
# CPU profiling (Linux perf)
perf record -g ./benchmark
perf report

# Cache analysis
perf stat -e cache-misses,cache-references ./benchmark

# Assembly inspection
objdump -d -M intel -C ./benchmark > asm.txt
```

### Common Bottlenecks

1. **Cache misses** - Use SoA layout for large datasets
2. **Branch misprediction** - Eliminate conditionals in hot loops
3. **False sharing** - Pad shared data to cache line boundaries
4. **Memory bandwidth** - Prefer in-place operations

---

## Future Architecture Direction

### Expression Templates (Planned)

```cpp
// Lazy evaluation
auto expr = A + B * C + D;
Vec3f result = expr.eval();  // Single pass, no temporaries
```

### Matrix Views (Planned)

```cpp
// Non-owning submatrix views
auto block = matrix.block<2, 2>(0, 0);
block = Mat2f::identity();  // Modifies original matrix
```

### Constexpr Everything (C++26)

```cpp
// Full compile-time linear algebra
constexpr Mat4f view = lookAt(eye, center, up);
constexpr Mat4f proj = perspective(fov, aspect, near, far);
constexpr Mat4f mvp = proj * view * model;
```

### SIMD Abstraction Layer

```cpp
// Unified SIMD API
using SimdFloat4 = Simd<float, 4>;  // NEON, SSE, AVX
SimdFloat4 a = load(ptr);
SimdFloat4 b = load(ptr + 4);
SimdFloat4 c = a * b + c;
store(result, c);
```

---

## Design Trade-offs

| Decision | Benefit | Cost | Rationale |
|----------|---------|------|-----------|
| Header-only | Easy integration | Compilation time | Embedded target priority |
| Stack allocation | Predictable perf | Size limitations | Real-time requirements |
| C++17 minimum | Modern constexpr and traits | Drops legacy compilers | Current embedded toolchains |
| Row-major | Cache-friendly rows | Column access slower | Math convention |
| Template-based | Type safety | Debug symbols large | Embedded safety critical |
| SIMD manual | Maximum control | Platform-specific code | Performance critical |

---

## References

- [PlantUML Diagrams](design/) - Visual architecture diagrams
- [PERFORMANCE.md](PERFORMANCE.md) - Detailed benchmarks
- [SAFETY_IMPROVEMENTS.md](SAFETY_IMPROVEMENTS.md) - Safety features
- [docs/SIMD_Optimizations.md](docs/SIMD_Optimizations.md) - SIMD implementation details

---

**Document Version:** 1.0
**Date:** January 31, 2026
**Authors:** MicroLA Development Team
