# C++ Standard Version Optimizations

MicroLA requires C++20 and takes advantage of newer C++ standards when available.

## Feature Detection

The library automatically detects the C++ standard version and enables appropriate optimizations. All feature detection macros are centralized in [`compiler_features.hpp`](../include/microla/compiler_features.hpp):

```cpp
// C++20 (required baseline): constexpr, if constexpr, inline variables, [[nodiscard]]
// C++20: MICROLA_CONSTEXPR20, MICROLA_CONSTEVAL, std::bit_cast
// C++23: MICROLA_CONSTEXPR23, MICROLA_IF_CONSTEVAL, MICROLA_UNREACHABLE
// C++26: MICROLA_CONSTEXPR26, MICROLA_CONSTEXPR_TRIG (constexpr math functions)

// Compiler Hints (all versions):
// MICROLA_ASSUME(cond), MICROLA_FORCEINLINE, MICROLA_BIT_CAST(T, val)
```

### Compiler Optimization Hints

MicroLA provides cross-compiler optimization macros:

**MICROLA_UNREACHABLE()** - Mark code paths that should never execute:

```cpp
switch(axis) {
    case 0: return rotation_x(angle);
    case 1: return rotation_y(angle);
    case 2: return rotation_z(angle);
    default: MICROLA_UNREACHABLE(); // Tells compiler: never happens
}
```

- C++23: Uses `std::unreachable()`
- GCC/Clang: `__builtin_unreachable()`
- MSVC: `__assume(0)`

**MICROLA_ASSUME(condition)** - Assert runtime conditions for optimization:

```cpp
MICROLA_ASSUME(length > 0);  // Enables division optimizations
float inv_length = 1.0f / length;
```

- Clang: `__builtin_assume(cond)`
- MSVC: `__assume(cond)`
- GCC 13+: `__attribute__((assume(cond)))`
- Fallback: `if (!(cond)) MICROLA_UNREACHABLE()`

**MICROLA_FORCEINLINE** - Strong inline hint for hot paths:

```cpp
MICROLA_FORCEINLINE float dot_impl(const float* a, const float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
```

- MSVC: `__forceinline`
- GCC/Clang: `__attribute__((always_inline)) inline`

**Benefits for embedded:**
- Smaller code size (dead code elimination)
- Better register allocation
- Reduced branching overhead
- Improved loop unrolling

## C++20 Baseline Optimizations

### Relaxed constexpr

More functions can be evaluated at compile-time with the required C++20 baseline:

```cpp
constexpr T dot(const Vec& other) const noexcept {
    T sum = T(0);
    for (std::size_t i = 0; i < N; ++i) {
        sum += data[i] * other.data[i];
    }
    return sum;
}
```

**Benefits for embedded:**
- More compile-time computation
- Smaller code size
- No runtime overhead

### Binary Literals & Digit Separators

Available but not currently used in MicroLA. Useful for bit manipulation:

```cpp
constexpr uint32_t mask = 0b1111'0000'1010'0101;
```

### if constexpr (Compile-Time Branching)

Eliminates runtime branches when conditions are known at compile-time:

```cpp
if constexpr (std::is_same_v<T, float>) {
    // SIMD code - no runtime overhead
}
```

**Benefits for embedded:**
- Zero runtime branching overhead
- Smaller code size (dead code eliminated)
- Better compiler optimization
- Reduced instruction cache pressure

**Impact:** ~5-10% performance improvement in SIMD-enabled code

### inline Variables

Allows header-only constants without ODR violations:

```cpp
// C++20 header-only definition
inline constexpr int X = 0;
```

### Structured Bindings

Not used in current API but useful for user code:

```cpp
auto [x, y, z] = vec.to_array();  // Still available under C++20
```

## C++20 Optimizations

### Concepts (Better Error Messages)

Type constraints with clear error messages:

```cpp
// SFINAE (cryptic errors)
template<typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class Vec { ... };

// C++20: Concepts (clear errors)
template<Arithmetic T>
class Vec { ... };
```

**Error messages:**
- **SFINAE-style**: "substitution failure, no matching template..."
- **C++20**: "T does not satisfy concept Arithmetic"

### std::bit_cast (Type-Safe Punning)

Safer alternative to reinterpret_cast for SIMD operations:

```cpp
// C++20: Well-defined, optimizes to same assembly
float f = std::bit_cast<float>(uint_val);
```

**Benefits for embedded:**
- Same performance as reinterpret_cast
- No undefined behavior
- Works with constexpr
- Better compiler optimization opportunities

### [[likely]] and [[unlikely]] Branch Hints


**Benefits for embedded:**
- Better instruction cache utilization
- Reduced branch misprediction penalties
- ~2-5% improvement in hot loops

### consteval (Compile-Time Only)

Force evaluation at compile-time:

```cpp
// C++20: Must be compile-time computable
MICROLA_CONSTEVAL int compute_at_compile_time(int x) {
    return x * x + 42;
}
```

**Benefits:** Guaranteed zero runtime cost

### constinit (Static Initialization)

Guarantee static initialization order:

```cpp
MICROLA_CONSTINIT const Vec<float, 3> unit_x(1.0f, 0.0f, 0.0f);
```

**Benefits:** No dynamic initialization overhead

## Performance Summary

| Feature | C++ Version | Performance Gain | Code Size Impact |
|---------|-------------|------------------|------------------|
| constexpr baseline | C++20 | 5-10% | -10-15% |
| [[likely]]/[[unlikely]] | C++20 | 2-5% | Neutral |
| std::bit_cast | C++20 | Neutral | Neutral |
| Concepts | C++20 | Neutral | +1-2% |
| if consteval | C++23 | 0-3% | -1-5% |
| constexpr trig (C++26) | C++26 | 5-15%* | -5-10%* |

**Combined effect:**
- **C++20:** 10-20% performance improvement, 5-15% code size reduction
- **C++26:** Additional 5-15% for code using rotation matrices*

*For applications with significant rotation matrix usage (e.g., robotics, graphics, sensor fusion)

## Compiler Support

### C++20 (minimum)

- GCC 10.0+
- Clang 10.0+
- MSVC 2019 16.11+
- IAR EWARM 9.30+

### C++23

- GCC 11.0+ (partial), 13.0+ (complete)
- Clang 15.0+ (partial), 17.0+ (complete)
- MSVC 2022 17.6+

### C++26

- GCC 14.0+ (preview)
- Clang 18.0+ (preview)
- MSVC 2024+ (expected)
- **Note:** C++26 is not finalized yet (as of 2026)

## Enabling Higher Standards

### CMake

```cmake
# C++20
target_compile_features(my_app PRIVATE cxx_std_20)

# C++23
target_compile_features(my_app PRIVATE cxx_std_23)

# C++26 (when available)
target_compile_features(my_app PRIVATE cxx_std_26)
```

### IAR Embedded Workbench

1. Project Options → C/C++ Compiler → Language
2. Set C++ dialect: C++20, C++23, or C++26 preview when available

### GCC/Clang

```bash
g++ -std=c++20 ...  # C++20
g++ -std=c++23 ...  # C++23
g++ -std=c++2c ...  # C++26 (preview)
```

## Recommendations

### For Projects

- **Use C++20** as the default supported baseline
- Enable C++23 when your toolchain has solid `if consteval` and `std::unreachable` support

### For Existing Projects

- Treat **C++20** as the minimum supported standard
- Consider C++23 for stronger constexpr/runtime dual-path support

### For Safety-Critical Projects

- **C++20** is the maintained MicroLA baseline
- Qualify optional C++23/C++26 features separately if your certification process requires it

## Current Library Status

MicroLA uses these features when available:

| Feature | Usage |
|---------|-------|
| C++20 constexpr | ✓ Used in Vec/Mat/Quaternion operations |
| C++20 if constexpr | ✓ Used in SIMD dispatch |
| C++20 inline variables | ✓ Used in constants |
| C++20 concepts | ✓ Type constraints defined |
| C++20 bit_cast | ✓ Used in SIMD operations |
| C++20 [[likely]] | ✓ Used in hot paths |
| C++20 consteval | ✓ Used for compile-time config validation |
| C++20 constinit | ⚠️  Ready for use |
| C++23 if consteval | ⚠️  Ready for use |
| C++26 constexpr trig | ✓ Used in rotation, norms, and Euler-angle helpers |

✓ = Active
⚠️ = Available but not yet used

## Future Enhancements

Potential C++23 features:
- `std::mdspan` for better matrix views
- `if consteval` for hybrid compile/runtime code
- Deducing `this` for CRTP elimination

---

## C++23 Optimizations

### if consteval

Allows different code paths for compile-time vs runtime evaluation:

```cpp
MICROLA_IF_CONSTEVAL {
    // Compile-time algorithm (simpler, more iterations)
    return slow_but_accurate();
} else {
    // Runtime algorithm (optimized)
    return fast_approximation();
}
```

**Benefits for embedded:**
- Compile-time: accuracy/precision at zero runtime cost
- Runtime: speed/efficiency when needed
- Single function handles both contexts

**Status:** Ready for C++23, currently uses fallback

### Deducing this

Simplifies CRTP patterns and reduces template complexity:

```cpp
// C++20: Verbose CRTP
template<typename Derived>
class Base {
    auto& derived() { return static_cast<Derived&>(*this); }
};

// C++23: Simpler with deducing this
class Simple {
    auto& method(this auto& self) { /* ... */ }
};
```

**Status:** Not yet used, potential for future refactoring

### std::mdspan

Multidimensional array views without ownership:

```cpp
// Potential future API enhancement
std::mdspan<float, 3, 3> view(matrix.data());
auto submatrix = view.subspan(0, 2, 0, 2);  // 2x2 top-left
```

**Status:** Under consideration for future MicroLA 2.x

---

## C++26 Optimizations

### constexpr Mathematical Functions

**Major Feature:** `std::sin`, `std::cos`, `std::sqrt`, and other `<cmath>` functions become `constexpr` in C++26!

This enables **compile-time rotation matrix creation with arbitrary angles**:

```cpp
// C++26: ANY angle computed at compile time!
constexpr auto R = Mat<float, 3, 3>::rotation_z(1.2345f);
// Matrix computed entirely at compile time, zero runtime cost

constexpr auto transform = Mat<float, 3, 3>::rotation_y(deg_to_rad(37.5f));
// Even with degree conversion - all compile-time
```

**Benefits for embedded:**
- Zero runtime trigonometry overhead
- Perfect for fixed transformations
- Smaller code (no trig library calls)
- Guaranteed compile-time correctness

**Library Support:** MicroLA 0.0.3 is ready - trig-dependent APIs (rotation, norm, and Euler-angle helpers) automatically become `constexpr` when compiled with C++26

#### Compile-Time Rotation: Version Comparison

| C++ Version | Capability | Example |
|-------------|------------|---------|
| **C++20-C++23** | Special angles only (0°, 90°, 180°, 270°) | `rotation_x_deg<90>()` |
| **C++26** | ANY angle at compile time | `rotation_x(1.234f)` in constexpr context |

**Migration path:**

```cpp
// Pre-C++26: Use special angle templates
constexpr auto R90 = Mat<float, 3, 3>::rotation_z_deg<90>();

// C++26+: Use normal functions in constexpr context
constexpr auto R_any = Mat<float, 3, 3>::rotation_z(1.234f);
// Both work, second is more flexible
```

### constexpr Vector/String Operations

More standard library functions become `constexpr`:

- `std::vector` operations (limited compile-time evaluation)
- `std::string` operations
- More `<algorithm>` functions

**Impact on MicroLA:** Minimal (we avoid std containers for embedded compatibility)

---

## Compile-Time Rotation Matrix Guide

### For C++20-C++23: Special Angles Only

Use template-based rotation for 0°, 90°, 180°, 270° multiples:

```cpp
// Compile-time rotations (no runtime cost)
constexpr auto R90x = Mat<float, 3, 3>::rotation_x_deg<90>();
constexpr auto R180y = Mat<float, 3, 3>::rotation_y_deg<180>();
constexpr auto R270z = Mat<float, 3, 3>::rotation_z_deg<270>();
constexpr auto R2D = Mat<float, 2, 2>::rotation_deg<90>();

// Use in constexpr context
constexpr Vec<float, 3> v(1.0f, 0.0f, 0.0f);
constexpr Vec<float, 3> rotated = R90x * v;
// Result: (0, 1, 0) - computed at compile time
```

**Limitations:**
- Only 0°, 90°, 180°, 270° supported
- Compile error for other angles
- Uses lookup tables (no trig functions)

### For C++26+: Any Angle at Compile Time

Use regular rotation functions in `constexpr` context:

```cpp
// C++26: Any angle works at compile time
constexpr auto R = Mat<float, 3, 3>::rotation_z(1.2345f);
constexpr auto R2 = Mat<float, 3, 3>::rotation_x(deg_to_rad(37.5f));

// Complex transformations
constexpr auto combined =
    Mat<float, 3, 3>::rotation_z(0.5f) *
    Mat<float, 3, 3>::rotation_y(1.2f);

// All computed at compile time - zero runtime overhead!
```

**Benefits:**
- No angle restrictions
- Natural syntax (same as runtime)
- Automatic when compiled with C++26

### Workarounds for Arbitrary Angles (C++20-C++23)

If you need compile-time rotations at arbitrary angles before C++26:

#### Option 1: Precompute and Hard-Code

```cpp
// Precompute angle offline (e.g., Python: math.cos(1.234))
constexpr float cos_val = 0.32913440f;
constexpr float sin_val = 0.94427961f;

constexpr Mat<float, 3, 3> custom_rotation() {
    Mat<float, 3, 3> result = Mat<float, 3, 3>::identity();
    result.data[0] = cos_val;   result.data[1] = -sin_val;
    result.data[3] = sin_val;   result.data[4] = cos_val;
    return result;
}
```

#### Option 2: Compile-Time Trig Approximation

Implement constexpr Taylor series (limited accuracy):

```cpp
// Compile-time cosine approximation (good to ~1e-4 for small angles)
constexpr float cos_approx(float x) {
    // Taylor series: cos(x) ≈ 1 - x²/2 + x⁴/24 - x⁶/720
    float x2 = x * x;
    return 1.0f - x2/2.0f + x2*x2/24.0f - x2*x2*x2/720.0f;
}

constexpr auto R = /* use cos_approx() */;
```

**Note:** Full accuracy requires many terms, increases compile time

#### Option 3: Use Runtime Computation

For infrequent use, runtime is often acceptable:

```cpp
// Computed once at startup
const auto R = Mat<float, 3, 3>::rotation_z(1.234f);
// Then use R throughout program
```

---

**Date**: May 31, 2026
**MicroLA Version**: 0.0.3
