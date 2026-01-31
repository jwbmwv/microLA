# Contributing to MicroLA

Thank you for your interest in contributing to MicroLA! This document provides guidelines and instructions for contributors.

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Getting Started](#getting-started)
3. [Development Workflow](#development-workflow)
4. [Code Style Guidelines](#code-style-guidelines)
5. [Testing Requirements](#testing-requirements)
6. [Documentation Standards](#documentation-standards)
7. [Pull Request Process](#pull-request-process)
8. [Commit Message Guidelines](#commit-message-guidelines)
9. [Review Process](#review-process)

---

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help maintain a welcoming community
- Report issues to maintainers if needed

---

## Getting Started

### Prerequisites

- C++11 or later compiler (GCC 7+, Clang 6+, MSVC 2017+, IAR EWARM)
- CMake 3.13.1 or later
- Git for version control

### Setting Up Development Environment

```bash
# Clone the repository
git clone https://github.com/yourusername/microla.git
cd microla

# Create build directory
mkdir build && cd build

# Configure with tests enabled
cmake .. -DMICROLA_LINEAR_BUILD_TESTS=ON \
         -DMICROLA_LINEAR_BUILD_EXAMPLES=ON \
         -DMICROLA_LINEAR_BUILD_BENCHMARKS=ON

# Build
cmake --build .

# Run tests
ctest --output-on-failure
```

### Using CMake Presets (Recommended)

```bash
# List available presets
cmake --list-presets

# Configure with a preset
cmake --preset debug
cmake --preset release
cmake --preset coverage

# Build
cmake --build --preset debug

# Test
ctest --preset debug
```

---

## Development Workflow

### Branch Naming Convention

Use descriptive branch names following this pattern:

- `feature/<short-description>` - New features
- `fix/<issue-number>-<description>` - Bug fixes
- `docs/<description>` - Documentation updates
- `refactor/<description>` - Code refactoring
- `test/<description>` - Test additions/improvements
- `perf/<description>` - Performance improvements

**Examples:**
```
feature/svd-decomposition
fix/123-quaternion-normalization
docs/cookbook-kalman-filter
refactor/matrix-multiply-simd
test/property-based-testing
perf/cache-trig-values
```

### Creating a Feature Branch

```bash
# Update main branch
git checkout main
git pull origin main

# Create feature branch
git checkout -b feature/your-feature-name

# Make changes and commit
git add .
git commit -m "feat: add SVD decomposition"

# Push to your fork
git push origin feature/your-feature-name
```

---

## Code Style Guidelines

### General Principles

1. **Header-only library** - All code in headers
2. **Zero-overhead abstractions** - No runtime cost
3. **C++11 compatibility** - Support embedded systems
4. **Const-correctness** - Mark all immutable functions const
5. **Explicit over implicit** - Clear intent in code

### Formatting Standards

**Brace Style:** Allman (braces on following line)

```cpp
// ✅ Correct
if (condition)
{
    doSomething();
}

// ❌ Incorrect
if (condition) {
    doSomething();
}
```

**Indentation:** 4 spaces (no tabs)

**Line Length:** 100 characters maximum (120 acceptable for complex templates)

**Naming Conventions:**

```cpp
// Classes: PascalCase
class SquareMat { };

// Functions: camelCase
void transpose();
float normSquared() const;

// Variables: camelCase
float myVariable;
int rowCount;

// Constants: UPPER_SNAKE_CASE
constexpr float PI = 3.14159265358979323846f;

// Template parameters: PascalCase
template<typename T, size_t N>
class Vec { };

// Private members: prefix with m_
class MyClass
{
private:
    float m_data[4];
    int m_size;
};
```

**Automated Formatting:**

We use `clang-format` with the provided `.clang-format` file:

```bash
# Format all files
clang-format -i include/microla/*.hpp
clang-format -i tests/**/*.cpp
```

### C++11 Compatibility Macros

Always use compatibility macros for C++14/17/20 features:

```cpp
// ✅ Correct - Works in C++11
MICROLA_CONSTEXPR float compute()
{
    return 42.0f;
}

// ❌ Incorrect - C++14+ only
constexpr float compute()
{
    return 42.0f;
}
```

**Available Macros:**
- `MICROLA_CONSTEXPR` - constexpr for C++14+, inline for C++11
- `MICROLA_IF_CONSTEXPR` - if constexpr for C++17+, if for earlier
- `MICROLA_NODISCARD` - [[nodiscard]] for C++17+

### Template Code Guidelines

```cpp
// Use size_t for dimensions
template<typename T, size_t N>
class Vec { };

// Forward declare when possible
template<typename T, size_t N> class Vec;

// Document template parameters
/// \tparam T Scalar type (float, double)
/// \tparam R Number of rows
/// \tparam C Number of columns
template<typename T, size_t R, size_t C>
class Mat { };
```

### SIMD Code Guidelines

```cpp
// Always provide scalar fallback
#if defined(CONFIG_MICROLA_NEON)
    // NEON implementation
    float32x4_t result = vmulq_f32(a, b);
#else
    // Scalar fallback
    for (size_t i = 0; i < 4; ++i)
    {
        result[i] = a[i] * b[i];
    }
#endif

// Use platform detection macros
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define HAS_NEON 1
#endif
```

### Debug Assertions

```cpp
// Use MICROLA_DEBUG for bounds checking
T& at(size_t index)
{
#ifdef MICROLA_DEBUG
    assert(index < N && "Vec::at() - Index out of bounds");
#endif
    return data[index];
}
```

---

## Testing Requirements

### Test Coverage Requirements

- **New features:** 100% line coverage
- **Bug fixes:** Test reproducing the bug + regression test
- **Refactoring:** Existing tests must pass

### Writing Tests

Use Google Test for unit tests:

```cpp
TEST(MatrixTest, Transpose)
{
    Mat<float, 2, 3> m = {
        1, 2, 3,
        4, 5, 6
    };
    
    auto mt = m.transpose();
    
    EXPECT_FLOAT_EQ(mt(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(mt(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(mt(2, 0), 3.0f);
}
```

### Test Organization

```
tests/
├── google/
│   ├── test_vec.cpp          # Vector operations
│   ├── test_mat.cpp          # Matrix operations
│   ├── test_quat.cpp         # Quaternion operations
│   ├── test_edge_cases.cpp   # Edge cases, NaN, Inf
│   ├── test_constexpr.cpp    # Compile-time evaluation
│   └── test_simd_equivalence.cpp  # SIMD correctness
└── zephyr/
    └── src/
        └── test_*.cpp        # Embedded platform tests
```

### SIMD Equivalence Testing

New SIMD code must pass equivalence tests:

```cpp
TEST(SIMDEquivalence, VectorAdd)
{
    Vec3f a(1, 2, 3);
    Vec3f b(4, 5, 6);
    
    Vec3f result = a + b;
    
    // Results must match scalar within epsilon
    EXPECT_FLOAT_EQ(result.x(), 5.0f);
    EXPECT_FLOAT_EQ(result.y(), 7.0f);
    EXPECT_FLOAT_EQ(result.z(), 9.0f);
}
```

### Running Tests Locally

```bash
# All tests
ctest --output-on-failure

# Specific test
ctest -R MatrixTest

# With verbose output
ctest -V

# Run under sanitizers
cmake .. -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"
make
ctest
```

---

## Documentation Standards

### Code Documentation (Doxygen)

All public APIs must have Doxygen comments:

```cpp
/// \brief Computes the dot product of two vectors
///
/// \param other The vector to compute dot product with
/// \return The scalar dot product
///
/// \note Returns 0 if vectors are perpendicular
/// \warning Result may overflow for large vectors
///
/// **Example:**
/// \code{.cpp}
/// Vec3f a(1, 0, 0);
/// Vec3f b(0, 1, 0);
/// float dot = a.dot(b);  // Returns 0
/// \endcode
///
/// **Performance:** O(N) scalar operations
///
/// **SIMD:** Optimized with ARM NEON when available
MICROLA_CONSTEXPR T dot(const Vec<T, N>& other) const
{
    // Implementation
}
```

### Documentation Files

When adding features, update:

1. **API_Documentation.md** - API reference
2. **COOKBOOK.md** - Practical examples
3. **TROUBLESHOOTING.md** - Common issues
4. **PERFORMANCE.md** - Benchmark results
5. **README.md** - If adding major feature

### Example Code

Include complete, runnable examples:

```cpp
// ✅ Complete example
#include <microla/microla.hpp>

int main()
{
    using namespace microla;
    
    Vec3f v(1, 2, 3);
    float len = v.norm();
    
    return 0;
}

// ❌ Incomplete fragment
Vec3f v(1, 2, 3);
float len = v.norm();
```

---

## Pull Request Process

### Before Submitting

- [ ] Code follows style guidelines (run `clang-format`)
- [ ] All tests pass (`ctest`)
- [ ] New tests added for new features
- [ ] Documentation updated
- [ ] No compiler warnings
- [ ] CHANGELOG.md updated (for features/breaking changes)
- [ ] Benchmarks run (for performance changes)

### PR Template

When creating a PR, include:

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix (non-breaking change fixing an issue)
- [ ] New feature (non-breaking change adding functionality)
- [ ] Breaking change (fix or feature causing existing functionality to change)
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Refactoring (no functional changes)

## Testing
- [ ] Unit tests added/updated
- [ ] All tests pass
- [ ] Tested on ARM NEON (if applicable)
- [ ] Tested on embedded target (if applicable)

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments added to hard-to-understand areas
- [ ] Documentation updated
- [ ] No new warnings generated
- [ ] Dependent changes merged

## Performance Impact
[If performance-related, include benchmark results]

## Breaking Changes
[List any breaking API changes]

## Related Issues
Fixes #123
Relates to #456
```

### PR Title Format

Use conventional commits format:

```
feat: add SVD decomposition
fix: correct quaternion slerp edge case
docs: add Kalman filter cookbook example
perf: optimize 4x4 matrix multiplication with NEON
refactor: extract common matrix operations
test: add property-based testing framework
```

---

## Commit Message Guidelines

Follow [Conventional Commits](https://www.conventionalcommits.org/):

### Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Types

- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, no logic change)
- `refactor`: Code refactoring
- `perf`: Performance improvements
- `test`: Adding or updating tests
- `build`: Build system changes
- `ci`: CI/CD changes
- `chore`: Other changes (dependencies, etc.)

### Examples

```
feat(matrix): add LU decomposition with partial pivoting

Implements LU decomposition using Doolittle algorithm with
partial pivoting for numerical stability.

Performance: O(n³) time, O(1) space
Tested on matrices up to 100x100

Closes #234
```

```
fix(quaternion): handle edge case in slerp for opposite quaternions

When quaternions are nearly opposite (dot product < -0.95),
slerp would produce invalid results. Now falls back to
linear interpolation through orthogonal quaternion.

Fixes #456
```

```
perf(simd): cache sin/cos values in rotation matrices

Reduces redundant trigonometric calculations by ~40% in
repeated rotation operations.

Benchmark improvements:
- rotateX: 2.3ns → 1.4ns (39% faster)
- rotateY: 2.3ns → 1.4ns (39% faster)
- rotateZ: 2.3ns → 1.4ns (39% faster)
```

---

## Review Process

### For Contributors

1. **Wait for CI** - All checks must pass before review
2. **Address feedback** - Respond to all review comments
3. **Request re-review** - After addressing feedback
4. **Be patient** - Maintainers review in priority order

### What Reviewers Look For

- **Correctness** - Does it work as intended?
- **Performance** - Any regressions or improvements?
- **Safety** - Bounds checking, overflow handling?
- **Compatibility** - C++11 compliance maintained?
- **Tests** - Adequate coverage?
- **Documentation** - Clear and complete?
- **Style** - Follows guidelines?
- **API design** - Consistent with existing API?

### Review Response Times

- **Bug fixes** - Within 48 hours
- **Features** - Within 1 week
- **Documentation** - Within 3 days

---

## Additional Resources

### Documentation

- [Quick Reference](QUICK_REFERENCE.md)
- [Architecture Guide](ARCHITECTURE.md)
- [Cookbook](docs/COOKBOOK.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Performance Guide](PERFORMANCE.md)

### Communication

- **Issues** - Bug reports and feature requests
- **Discussions** - Questions and general discussion
- **Pull Requests** - Code contributions

### Build Configurations

```bash
# Debug build with sanitizers
cmake --preset debug-sanitize

# Release build with benchmarks
cmake --preset release-bench

# Coverage analysis
cmake --preset coverage
cmake --build --preset coverage
ctest --preset coverage
```

---

## Recognition

Contributors are recognized in:

- **README.md** - Contributors section
- **CHANGELOG.md** - Per-version credits
- **Git history** - All commits preserved

Thank you for contributing to MicroLA! 🎉
