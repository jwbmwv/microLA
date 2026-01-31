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

- C++17 or later compiler (GCC 7+, Clang 6+, MSVC 2017+, IAR EWARM)
- CMake 3.13.1 or later
- Git for version control

### Setting Up Development Environment

```bash
# Clone the repository
git clone https://github.com/jwbmwv/microla.git
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

# Common local development presets
cmake --preset debug
cmake --preset release

# Canonical automation-oriented presets
cmake --preset coverage-ci
cmake --preset sanitizers-ci
cmake --preset benchmark-ci

# Embedded profile presets
cmake --preset embedded-examples
cmake --preset embedded-tests

MicroLA enforces a baseline set of automated static-analysis checks. The full baseline and the deviation process are documented in `COMPLIANCE.md`.

- CI runs `clang-tidy` with checks: `cppcoreguidelines-*`, `bugprone-*`, `modernize-*`, `performance-*`, `readability-*`, and `portability-*`.
- CI runs `cppcheck` (`warning,performance,portability`) with `--force` so the header set is checked across more preprocessor configurations.
- `clang-format` is enforced via `scripts/format.sh` and CI formatting checks.

If a rule requires an exception, include a short justification in the PR, add an inline comment next to the deviation, and record the deviation in `COMPLIANCE.md`.

---

## Pull Request Process
ctest --preset debug
```

---

## Development Workflow

### Branch Naming Convention

Use descriptive branch names following this pattern:

- `feature/<short-description>` - Features
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
3. **C++17 baseline** - Support modern embedded toolchains
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

### Language Feature Usage

Use native C++17 language features directly in new code (`constexpr`, `if constexpr`, `[[nodiscard]]`, and standard type traits).

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

- **Features:** 100% line coverage
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

SIMD code must pass equivalence tests:

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
- [ ] Unit tests included where appropriate
- [ ] Documentation revised
- [ ] No compiler warnings
- [ ] CHANGELOG.md maintained (for feature/breaking changes)
- [ ] Benchmarks run (for performance changes)

### PR Template

When creating a PR, include:

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix (non-breaking change fixing an issue)
- [ ] Feature (non-breaking change adding functionality)
- [ ] Breaking change (fix or feature causing existing functionality to change)
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Refactoring (no functional changes)

## Testing
- [ ] Unit tests included or modified
- [ ] All tests pass
- [ ] Tested on ARM NEON (if applicable)
- [ ] Tested on embedded target (if applicable)

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments for hard-to-understand areas
- [ ] Documentation revised
- [ ] No additional warnings generated
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

- `feat`: Feature
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
- **Compatibility** - C++17 baseline maintained?
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
# Canonical sanitizer build
cmake --preset sanitizers-ci

# Local benchmark build with host-tuned flags
cmake --preset benchmark

# CI-style benchmark build
cmake --preset benchmark-ci

# Coverage analysis
cmake --preset coverage-ci
cmake --build --preset coverage-ci
ctest --preset coverage-ci
```

---

## Intellectual Property and Developer Certificate of Origin

### Project AI Generation Context

This project was substantially generated with AI coding assistance. See [NOTICE](NOTICE) for
the full disclosure. Contributors should be aware of this context when submitting changes.

### Developer Certificate of Origin (DCO)

By submitting a pull request or patch, you certify that:

1. **You authored the contribution** — it is your original work, or you have the right to
   submit it under the same license (Apache 2.0).
2. **You disclose AI-assisted contributions** — if your contribution was substantially
   generated by an AI tool, you must state this clearly in your pull request description,
   including which tool(s) were used.
3. **You have reviewed AI-generated code** — if using AI assistance, you have reviewed,
   understood, and accept responsibility for the submitted code and its correctness.
4. **No known IP conflicts** — to the best of your knowledge, your contribution does not
   infringe any third-party copyright, patent, or trade secret.
5. **License compatibility** — you license your contribution under the Apache License 2.0
   (SPDX-License-Identifier: Apache-2.0) and grant the rights described therein.

By including `Signed-off-by: Your Name <your.email@example.com>` in your commit message,
you certify the above. Example:

```
git commit -s -m "feat: add SVD decomposition"
```

### AI Contribution Policy

- AI-generated contributions are **accepted** provided they are reviewed, tested, and
  accompanied by a disclosure statement in the PR description.
- Contributors are personally responsible for the accuracy, correctness, and IP status
  of AI-generated code they submit.
- Do **not** submit code generated by tools whose terms of service prohibit submission
  to open source projects.
- Contributions that substantially reproduce copyrighted third-party code (whether
  AI-generated or otherwise) will be rejected.

### No Warranty from Maintainers

Maintainers make no warranty about the IP status of existing AI-generated code in this
repository or of accepted contributions. See [NOTICE](NOTICE) and the Apache License 2.0
disclaimer (Section 7) for the full liability and warranty terms.

---

## Recognition

Contributors are recognized in:

- **README.md** - Contributors section
- **CHANGELOG.md** - Per-version credits
- **Git history** - All commits preserved

Thank you for contributing to MicroLA! 🎉
