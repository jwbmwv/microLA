# MicroLA Troubleshooting Guide

Common issues, solutions, and debugging strategies for MicroLA.

## Table of Contents

1. [Compilation Issues](#compilation-issues)
2. [Linker Errors](#linker-errors)
3. [Runtime Problems](#runtime-problems)
4. [Performance Issues](#performance-issues)
5. [SIMD/Platform-Specific Issues](#simdplatform-specific-issues)
6. [Build System Integration](#build-system-integration)
7. [Debugging Techniques](#debugging-techniques)

---

## Compilation Issues

### Error: "no member named 'MICROLA_CONSTEXPR' in namespace 'microla'"

**Cause**: Using a C++ standard below C++20.

**Solution**:

```cmake
# CMakeLists.txt
target_compile_features(your_target PRIVATE cxx_std_20)

# Or set compiler flag
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

```bash
# GCC/Clang command line
g++ -std=c++20 main.cpp

# MSVC
cl /std:c++20 main.cpp
```

---

### Error: "cannot convert 'microla::Vec<float, 3>' to 'float*'"

**Cause**: MicroLA types are not implicitly convertible to raw pointers.

**Solution**:

```cpp
// ❌ Wrong
Vec3f v(1, 2, 3);
float* ptr = v;

// ✅ Correct
Vec3f v(1, 2, 3);
float* ptr = v.data();  // Explicit conversion

// Or use indexing
for (int i = 0; i < 3; ++i) {
    float value = v[i];
}
```

---

### Error: "call to constexpr function is not a constant expression"

**Cause**: Using trigonometric functions in constexpr context with C++ < 26.

**Solution**:

```cpp
// ❌ Wrong (only works in C++26)
constexpr auto m = Mat3f::rotation(45.0f * constants::deg_to_rad<float>);

// ✅ Correct for C++20-23 (use special angle helpers)
constexpr auto m = Mat3f::rotation_45();  // Precomputed
constexpr auto m2 = Mat3f::rotation_90();

// ✅ Or compute at runtime
const auto m3 = Mat3f::rotation(45.0f * constants::deg_to_rad<float>);
```

---

### Warning: "unused variable" for temporaries

**Cause**: Compiler warning on intermediate calculations.

**Solution**:

```cpp
// ❌ Triggers warning
Vec3f temp = a + b;

// ✅ Mark as intentionally unused (C++20)
[[maybe_unused]] Vec3f temp = a + b;

// ✅ Or use directly
Vec3f result = (a + b).normalized();

// ✅ Or suppress via attribute
#ifdef __GNUC__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-variable"
#endif
    Vec3f temp = a + b;
#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif
```

---

### Error: "no matching function for call to 'Mat<float, 4, 4>::Mat(int)'"

**Cause**: Ambiguous constructor overload.

**Solution**:

```cpp
// ❌ Wrong (ambiguous)
Mat4f m(0);

// ✅ Correct (explicit zero initialization)
Mat4f m = Mat4f();  // Default constructor (zero-initialized)

// ✅ Or use named constructor
Mat4f m = Mat4f::zero();

// ✅ Or identity
Mat4f m = Mat4f::identity();
```

---

### Compilation time too long (>60 seconds)

**Cause**: Template instantiation overhead, especially with heavy SIMD usage.

**Solutions**:

1. **Use forward declarations**:

```cpp
// header.hpp
namespace microla { template<typename T, int N> class Vec; }
class MyClass {
    microla::Vec<float, 3>* velocity;  // Pointer avoids full definition
};

// source.cpp
#include <microla/vec3D.hpp>
// Implementation here
```

2. **Split large files**:

```cpp
// Don't include everything
// ❌ #include <microla/microla.hpp>

// ✅ Include only what you need
#include <microla/vec3D.hpp>
#include <microla/matrix3D.hpp>
```

3. **Use precompiled headers** (CMake):

```cmake
target_precompile_headers(your_target PRIVATE
    <microla/microla.hpp>
    <microla/quaternion.hpp>
)
```

4. **Reduce template instantiations**:

```cpp
// ❌ Many instantiations
void process(Vec<float, 3> v);
void process(Vec<double, 3> v);
void process(Vec<int, 3> v);

// ✅ Single template
template<typename T>
void process(Vec<T, 3> v);
```

---

## Linker Errors

### Error: "undefined reference to 'microla::version()'"

**Cause**: Header-only library incorrectly linked or missing inline specifiers.

**Solution**:

```cmake
# CMakeLists.txt
# ❌ Wrong (tries to link)
target_link_libraries(your_target microla.a)

# ✅ Correct (header-only)
target_link_libraries(your_target INTERFACE microla::microla)

# Or just include directories
target_include_directories(your_target PRIVATE ${MICROLA_INCLUDE_DIR})
```

---

### Error: "multiple definition of 'microla::constants::pi'"

**Cause**: Building some translation units below the required C++20 mode or mixing inconsistent dialect settings.

**Solution**:

```cpp
// If using C++20+, this is automatic

// For C++20+, prefer inline constexpr constants in headers:
namespace microla {
namespace constants {
namespace {  // Anonymous namespace
    constexpr float pi = 3.14159265358979323846f;
}
}
}
```

Or use linker option:

```bash
# GCC/Clang
g++ -Wl,--allow-multiple-definition

# MSVC
cl /FORCE:MULTIPLE
```

---

### Error: "undefined reference to '__aeabi_uldivmod'" (ARM Cortex-M)

**Cause**: Missing runtime library for 64-bit division.

**Solution**:

```cmake
# Link against compiler runtime
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    target_link_libraries(your_target PRIVATE gcc)
endif()

# Or avoid 64-bit operations on Cortex-M
# Use float instead of double
Vec3f v;  // ✅ 32-bit
Vec3d v;  // ❌ 64-bit (requires soft float library)
```

---

## Runtime Problems

### Segmentation fault / Access violation

**Likely causes**:

1. **Out-of-bounds access**:

```cpp
Vec3f v;
float x = v[5];  // ❌ Undefined behavior (only 0-2 valid)

// ✅ Use bounds-checked access in debug builds
#define MICROLA_DEBUG
float x = v.at(5);  // Triggers assertion
```

2. **Uninitialized matrix**:

```cpp
Mat4f m;  // Default constructor zeros everything (safe)
// But if using uninitialized stack memory:
Mat4f* m = (Mat4f*)malloc(sizeof(Mat4f));  // ❌ Uninitialized
```

3. **Dangling reference**:

```cpp
// ❌ Wrong
const Vec3f& get_vec() {
    Vec3f temp(1, 2, 3);
    return temp;  // Returns reference to destroyed object
}

// ✅ Correct
Vec3f get_vec() {
    return Vec3f(1, 2, 3);  // Return by value (RVO optimizes)
}
```

---

### NaN (Not-a-Number) propagation

**Cause**: Division by zero or sqrt of negative number.

**Debug**:

```cpp
#include <cmath>

Vec3f v = some_computation();
if (std::isnan(v[0]) || std::isnan(v[1]) || std::isnan(v[2])) {
    // NaN detected - trace backward
}

// Enable floating-point exceptions (GCC/Clang)
#include <fenv.h>
feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
```

**Prevention**:

```cpp
// Use safe operations
Vec3f v = vec.normalized();  // Automatically checks for zero length

// Manual check
if (vec.length() > std::numeric_limits<float>::epsilon()) {
    Vec3f normalized = vec.normalized();
}
```

---

### Incorrect rotation results

**Common mistakes**:

1. **Wrong rotation order**:

```cpp
// Rotation order matters!
// ❌ Wrong (roll-pitch-yaw is Z-Y-X)
Mat3f rot = Mat3f::rotation_z(yaw) * Mat3f::rotation_y(pitch) * Mat3f::rotation_x(roll);

// ✅ Correct
Mat3f rot = Mat3f::rotation_x(roll) * Mat3f::rotation_y(pitch) * Mat3f::rotation_z(yaw);
```

2. **Degrees vs radians**:

```cpp
// ❌ Wrong (expects radians)
Mat3f rot = Mat3f::rotation_z(45.0f);

// ✅ Correct
Mat3f rot = Mat3f::rotation_z(45.0f * constants::deg_to_rad<float>);
```

3. **Quaternion multiplication order**:

```cpp
// Quaternion multiplication is NOT commutative
Quaternion<float> q1 = Quaternion<float>::from_axis_angle(Vec3f(1, 0, 0), angle1);
Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec3f(0, 1, 0), angle2);

// q1 * q2 applies q2 first, then q1 (right-to-left)
// q2 * q1 applies q1 first, then q2 (right-to-left)
```

---

### Matrix inversion returns identity matrix

**Cause**: Singular or near-singular matrix (determinant ≈ 0).

**Solution**:

```cpp
Mat4f m = /* ... */;
float det = m.determinant();

if (std::abs(det) < 1e-6f) {
    // Matrix is singular or ill-conditioned
    // Cannot invert reliably
} else {
    Mat4f inv = m.inverse();
}

// Check condition number (how well-conditioned the matrix is)
// High condition number = numerical instability
```

---

## Performance Issues

### Operations slower than expected

**Diagnostic steps**:

1. **Check optimization flags**:

```bash
# ❌ Debug build (slow)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# ✅ Release build
cmake -DCMAKE_BUILD_TYPE=Release ..

# ✅ Aggressive optimization
g++ -O3 -march=native -ffast-math main.cpp
```

2. **Verify SIMD is enabled**:

```cpp
#include <microla/compiler_features.hpp>

#ifdef CONFIG_MICROLA_NEON
    // NEON enabled
#else
    // Scalar fallback (slower)
#endif
```

3. **Profile hot paths**:

```bash
# GCC/Clang
g++ -O3 -g -pg main.cpp
./a.out
gprof a.out gmon.out > profile.txt

# Or use perf (Linux)
perf record ./a.out
perf report
```

4. **Avoid unnecessary copies**:

```cpp
// ❌ Slow (copies vector)
Vec3f transform(Vec3f v, const Mat3f& m) {
    return m * v;
}

// ✅ Fast (pass by const reference)
Vec3f transform(const Vec3f& v, const Mat3f& m) {
    return m * v;
}
```

5. **Use constexpr for constants**:

```cpp
// ❌ Runtime computation
Mat4f proj = Mat4f::perspective(60.0f * constants::deg_to_rad<float>, 1.0f, 0.1f, 100.0f);

// ✅ Compile-time (C++20+)
constexpr Mat4f identity = Mat4f::identity();
```

---

### High memory usage

**Causes**:

1. **Large matrix temporaries**:

```cpp
// ❌ Creates many temporaries
Mat4f result = (a * b) * (c * d) * (e * f);

// ✅ Reuse storage
Mat4f temp1 = a * b;
Mat4f temp2 = c * d;
temp1 = temp1 * temp2;
result = temp1 * (e * f);
```

2. **Stack overflow with large matrices**:

```cpp
// ❌ May overflow stack
Mat<float, 1000, 1000> big;  // 4 MB on stack!

// ✅ Use heap for large matrices
auto big = std::make_unique<Mat<float, 1000, 1000>>();
```

---

## SIMD/Platform-Specific Issues

### NEON not detected on ARM

**Diagnostic**:

```cpp
#include <iostream>

int main() {
#ifdef CONFIG_MICROLA_NEON
    std::cout << "NEON enabled\n";
#else
    std::cout << "NEON not enabled\n";
#endif
}
```

**Solution**:

```cmake
# CMakeLists.txt
option(MICROLA_ENABLE_NEON "Enable NEON" ON)
if(MICROLA_ENABLE_NEON)
    target_compile_definitions(your_target PRIVATE CONFIG_MICROLA_NEON)

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)")
        target_compile_options(your_target PRIVATE -mfpu=neon)
    endif()
endif()
```

Or compiler flag:

```bash
arm-linux-gnueabihf-g++ -mfpu=neon -DCONFIG_MICROLA_NEON main.cpp
```

---

### SIMD results differ from scalar

**Cause**: Floating-point rounding differences.

**Solution**:

```cpp
// Don't compare exact equality
// ❌ Wrong
if (simd_result == scalar_result) { /* ... */ }

// ✅ Correct (epsilon comparison)
Vec3f diff = simd_result - scalar_result;
if (diff.length() < 1e-5f) {
    // Results are equivalent
}
```

---

### Illegal instruction / SIGILL on ARM Cortex-M

**Cause**: NEON instructions used on CPU without NEON support.

**Solution**:

```cmake
# Disable NEON for Cortex-M (no NEON)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "cortex-m")
    set(MICROLA_ENABLE_NEON OFF)
    set(MICROLA_ENABLE_CMSIS ON)  # Use CMSIS-DSP instead
endif()
```

---

## Build System Integration

### CMake: "Target 'microla' not found"

**Solution**:

```cmake
# Option 1: add_subdirectory
add_subdirectory(external/microla)
target_link_libraries(your_target PRIVATE microla::microla)

# Option 2: FetchContent
include(FetchContent)
FetchContent_Declare(microla
    GIT_REPOSITORY https://github.com/jwbmwv/microla.git
    GIT_TAG main
)
FetchContent_MakeAvailable(microla)
target_link_libraries(your_target PRIVATE microla::microla)

# Option 3: Manual include
target_include_directories(your_target PRIVATE ${MICROLA_ROOT}/include)
```

---

### Zephyr: "module 'microla' not found"

**Solution**:

```yaml
# west.yml
manifest:
  projects:
    - name: microla
      url: https://github.com/jwbmwv/microla.git
      revision: main
      path: modules/lib/microla
```

Then:

```bash
west update
```

In `prj.conf`:

```ini
CONFIG_MICROLA=y
CONFIG_MICROLA_NEON=y  # If ARM with NEON
```

---

### PlatformIO / Arduino integration

**Solution**:

```ini
; platformio.ini
[env:myboard]
lib_deps =
    https://github.com/jwbmwv/microla.git

build_flags =
    -std=c++20
    -I include
```

Or manually copy `include/microla/` to your project's `lib/` folder.

---

## Debugging Techniques

### Enable assertions

```cpp
// Add to build
#define MICROLA_DEBUG

// Or in CMake
target_compile_definitions(your_target PRIVATE MICROLA_DEBUG)
```

Now bounds checking is active:

```cpp
Vec3f v;
v.at(5);  // Assertion failure: index 5 >= size 3
```

---

### Print matrix contents

```cpp
#include <iostream>

void print_mat4(const Mat4f& m) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << m(r, c) << " ";
        }
        std::cout << "\n";
    }
}

void print_vec3(const Vec3f& v) {
    std::cout << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]\n";
}
```

---

### Sanitizer instrumentation

```cmake
# CMakeLists.txt
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,undefined")
```

```bash
# Run with sanitizers
ASAN_OPTIONS=check_initialization_order=1 ./your_program
```

Detects:
- Out-of-bounds access
- Use-after-free
- Memory leaks
- Undefined behavior (integer overflow, misaligned access, etc.)

---

### GDB debugging

```bash
gdb ./your_program

# Useful commands
(gdb) break main
(gdb) run
(gdb) print my_vec
(gdb) print my_vec.data()[0]
(gdb) print sizeof(my_mat)
(gdb) backtrace  # Show call stack on crash
```

---

## Common Error Messages

| Error | Likely Cause | Solution |
|-------|--------------|----------|
| `static assertion failed: Dimensions must match` | Mismatched matrix/vector sizes | Check dimensions: `Mat<float, 3, 4> * Vec<float, 4>` |
| `no matching function for call to 'cross'` | Cross product requires Vec3 | Use `Vec<T, 3>` only for cross product |
| `division by zero` | Normalizing zero-length vector | Check length before normalizing |
| `std::bad_alloc` | Out of memory | Reduce matrix sizes or use heap allocation |
| `pure virtual method called` | Destroyed object still referenced | Fix object lifetime |

---

## Getting Help

If you're still stuck:

1. **Check documentation**: [docs/README.md](README.md)
2. **Search issues**: https://github.com/jwbmwv/microla/issues
3. **Ask on discussions**: https://github.com/jwbmwv/microla/discussions
4. **File a bug report**: Provide minimal reproducible example

### Bug Report Template

```markdown
**MicroLA Version**: 0.0.2
**Compiler**: GCC 11.4 / MSVC 19.35 / Clang 14.0
**Platform**: x86-64 / ARM Cortex-A72 / STM32F4
**Build**: Debug / Release
**C++ Standard**: 11 / 14 / 17 / 20

**Description**:
[What's wrong?]

**Minimal Reproducible Example**:
```cpp

#include <microla/microla.hpp>

using namespace microla;

int main() {
    Vec3f v(1, 2, 3);
    // Bug happens here...
}

```

**Expected**: [What should happen?]
**Actual**: [What actually happens?]
**Compiler Output**: [Error messages]
```

---

**Date**: May 31, 2026
**MicroLA Version**: 0.0.2
