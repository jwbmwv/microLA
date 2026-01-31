# MicroLA Test Suite

Comprehensive test suite for MicroLA using Google Test framework.

## Test Structure

The test suite is organized into the following test files:

### Core Component Tests

1. **test_vector.cpp** - Vec<T,N> class tests
   - Construction and initialization
   - Arithmetic operations (+, -, *, /)
   - Dot and cross products
   - Length, normalization, and magnitude
   - Interpolation (lerp, cubic hermite, catmull-rom)
   - Element-wise operations
   - Projection and rejection
   - Swizzling and homogeneous coordinates
   - Static factory methods

2. **test_matrix.cpp** - Mat<T,R,C> class tests
   - Construction and accessors
   - Matrix arithmetic operations
   - Matrix-vector multiplication
   - Transpose, determinant, trace, inverse
   - Rotation matrices (2D, 3D, axis-angle)
   - Block operations (extraction, insertion)
   - Decompositions (LU, QR, Cholesky, SVD)
   - Eigenvalues
   - Matrix norms (Frobenius, infinity, one)
   - Pseudoinverse and condition number

3. **test_quaternion.cpp** - Quaternion<T> class tests
   - Construction (components, axis-angle)
   - Arithmetic operations
   - Conjugate and inverse
   - Normalization
   - Rotation operations
   - Conversion to/from matrices
   - Euler angles (roll, pitch, yaw)
   - SLERP interpolation
   - Angle between quaternions
   - Static factory methods

4. **test_geometry.cpp** - Geometric primitives tests
   - Ray (origin, direction, at())
   - Plane (distance, projection, intersection)
   - AABB (contains, intersects, merge, expand)
   - Sphere (contains, intersects)
   - Triangle (normal, area, centroid, intersection)
   - Frustum (culling tests)
   - Various intersection tests

5. **test_constants.cpp** - Mathematical constants tests
   - Pi and related constants
   - Euler's number (e)
   - Square roots
   - Golden ratio
   - Angle conversion factors

6. **test_integration.cpp** - Integration tests
   - Complete transformation pipelines
   - Quaternion-matrix equivalence
   - Ray tracing scenarios
   - Physics simulations
   - Camera systems
   - Collision detection
   - Linear algebra applications
   - Numerical stability tests

## Building the Tests

### Prerequisites

- CMake 3.14 or higher
- Google Test library
- C++17 or higher compiler

### Building

```bash
cd microla
mkdir build && cd build
cmake .. -DMICROLA_LINEAR_BUILD_TESTS=ON
cmake --build . --target microla_gtests
```

### Running Tests

Run all tests:

```bash
ctest --output-on-failure
```

Or run the test executable directly:

```bash
./microla_gtests
```

Run specific test suites:

```bash
./microla_gtests --gtest_filter=VectorTest.*
./microla_gtests --gtest_filter=MatrixTest.*
./microla_gtests --gtest_filter=QuaternionTest.*
```

Run with verbose output:

```bash
./microla_gtests --gtest_verbose
```

## Test Coverage

The current host-side validation surface is anchored by the `host-tests` preset, which now passes **517/517** tests in a clean C++17 build.

That host surface covers:

- ✅ Core vector, matrix, quaternion, and view operations
- ✅ Geometry primitives and intersection workflows
- ✅ Numerical stability, safe math, fast math, and decomposition paths
- ✅ Compiler features, SIMD helpers, and embedded-contract checks
- ✅ The public `sensor_fusion.hpp` API through dedicated fusion tests

The release presets extend that surface further by executing the maintained `sensor_fusion.cpp` example as `sensor_fusion_example_smoke`.

## Test Categories

### Functional Tests

- Verify correct behavior of all public APIs
- Test boundary conditions
- Validate mathematical properties

### Edge Case Tests

- Zero vectors and matrices
- Degenerate geometries
- Numerical limits
- Division by zero handling

### Type Tests

- Float and double precision
- Integer types where applicable
- Template instantiation

### Integration Tests

- Multi-component workflows
- Real-world usage scenarios
- Transformation pipelines
- Physics simulations

### Numerical Stability Tests

- Accumulated error in repeated operations
- Large angle rotations
- Matrix decomposition accuracy

## Continuous Integration

These tests are designed to run in CI/CD pipelines:

```yaml
# Example CI configuration
test:
  script:
    - mkdir build && cd build
    - cmake .. -DMICROLA_LINEAR_BUILD_TESTS=ON
    - cmake --build .
    - ctest --output-on-failure
```

## Adding Tests

To add test cases:

1. Create or modify a test file in `tests/google/`
2. Use Google Test macros: `TEST()`, `TEST_F()`, `EXPECT_*`, `ASSERT_*`
3. Follow the naming convention: `test_<component>.cpp`
4. Tests will be automatically discovered by CMake

Example:

```cpp
TEST(MyComponentTest, MyFeature) {
    MyComponent obj;
    EXPECT_EQ(obj.getValue(), expected_value);
}
```

## Test Fixtures

Use test fixtures for common setup:

```cpp
class MyTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Common initialization
    }
    // Shared data members
};

TEST_F(MyTest, TestCase) {
    // Use shared data
}
```

## Benchmarking

For performance testing, use the separate benchmark suite in `benchmarks/`:

```bash
cmake --build . --target microla_benchmarks
./microla_benchmarks
```

## Troubleshooting

### Tests Won't Build

- Ensure Google Test is installed: `apt-get install libgtest-dev` (Linux)
- Or let CMake fetch it: add `FetchContent` to CMakeLists.txt

### Tests Fail

- Check compiler warnings
- Verify C++ standard compatibility
- Check for platform-specific issues (endianness, floating-point)

### Performance Issues

- Build in Release mode: `cmake .. -DCMAKE_BUILD_TYPE=Release`
- Enable optimizations: `-O3` flag
- Check for SIMD support if available

## Contributing

When contributing tests:

1. Write clear, descriptive test names
2. Test one concept per test case
3. Use appropriate assertions (`EXPECT_*` vs `ASSERT_*`)
4. Add comments for complex test scenarios
5. Ensure tests are deterministic
6. Avoid dependencies between tests

## License

SPDX-License-Identifier: Apache-2.0

Copyright (c) 2026 James Baldwin
