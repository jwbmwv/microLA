# MicroLA Zephyr Tests

Complete test suite for MicroLA using Zephyr's ztest framework.

## Overview

This directory contains comprehensive tests for the MicroLA header-only linear algebra library running on Zephyr RTOS. The tests verify functionality across vectors, matrices, quaternions, and integrated operations.

## Test Suites

### 1. Vector Tests (`test_vector.cpp`)

- Construction (default, variadic, copy, array)
- Element access and component accessors
- Arithmetic operations (addition, subtraction, multiplication, division)
- Vector operations (dot product, cross product, magnitude, normalization)
- Comparison operators
- Compound assignment operators

### 2. Matrix Tests (`test_matrix.cpp`)

- Construction (default, identity, array, copy)
- Element access and dimension methods
- Arithmetic operations (addition, subtraction, scalar/matrix multiplication)
- Matrix-vector multiplication
- Matrix operations (transpose, determinant, inverse)
- Comparison operators

### 3. Quaternion Tests (`test_quaternion.cpp`)

- Construction and identity
- Arithmetic operations (addition, subtraction, multiplication)
- Quaternion operations (norm, normalization, conjugate, inverse)
- Rotation operations (from axis-angle, rotate vector, to rotation matrix)
- SLERP (spherical linear interpolation)
- Comparison operators

### 4. Integration Tests (`test_integration.cpp`)

- Graphics pipeline transformations
- Coordinate system transformations
- Robotics kinematics (forward kinematics, quaternion chains)
- Sensor fusion (weighted averaging, motion prediction)
- Physics simulations (reflection, projectile motion)
- Matrix-vector combined operations
- Orthonormal basis verification
- Constexpr operations

## Building and Running

### Prerequisites

1. **Zephyr SDK** installed and configured
2. **West** tool installed
3. **MicroLA** as a Zephyr module (see [../../zephyr/README.md](../../zephyr/README.md))

### Quick Start

#### 1. Using west (recommended)

```bash
# Navigate to the test directory
cd tests/zephyr

# Build for native_posix (runs on your host machine)
west build -b native_posix

# Run the tests
west build -t run
```

#### 2. For QEMU Cortex-M3

```bash
# Build for QEMU Cortex-M3
west build -b qemu_cortex_m3 -p

# Run in QEMU
west build -t run
```

#### 3. For other boards

```bash
# Build for your target board
west build -b <your_board> -p

# Flash to hardware (if available)
west flash

# Or run in emulator (if supported)
west build -t run
```

### Using Twister (Zephyr Test Runner)

Twister is Zephyr's test runner that can execute tests across multiple platforms:

```bash
# Run all MicroLA tests on available platforms
twister -T tests/zephyr

# Run on specific platform
twister -T tests/zephyr -p qemu_cortex_m3

# Run with verbose output
twister -T tests/zephyr -v

# Generate test report
twister -T tests/zephyr --report-name microla_test_report
```

## Configuration Options

Edit [prj.conf](prj.conf) to customize test behavior:

### C++ Standard Library (Required)

MicroLA requires the full C++ standard library (including `<type_traits>`, `<limits>`, etc.):

```conf
CONFIG_CPP=y                         # Enable C++ support
CONFIG_STD_CPP20=y                   # Use C++20 standard
CONFIG_REQUIRES_FULL_LIBCPP=y        # Required: Enable full C++ stdlib
CONFIG_EXCEPTIONS=n                  # Disable exceptions (embedded)
CONFIG_RTTI=n                        # Disable RTTI (embedded)
```

**Note:** By default, Zephyr uses a minimal C++ library. `CONFIG_REQUIRES_FULL_LIBCPP=y` is required to provide the standard headers MicroLA needs.

### Memory Configuration

```conf
CONFIG_MAIN_STACK_SIZE=4096          # Stack size for main thread
CONFIG_HEAP_MEM_POOL_SIZE=16384      # Heap size for dynamic allocations
```

### Floating Point Precision

```conf
CONFIG_NEWLIB_LIBC_FLOAT_PRINTF=y    # Enable float printf
CONFIG_NEWLIB_LIBC_FLOAT_SCANF=y     # Enable float scanf
```

### SIMD Optimizations

Uncomment in [prj.conf](prj.conf) to enable hardware acceleration:

```conf
# ARM Cortex-M with CMSIS-DSP
CONFIG_CMSIS_DSP=y
CONFIG_MICROLA_CMSIS=y

# ARM Cortex-A or ARM64 with NEON
CONFIG_MICROLA_NEON=y

# ARM Cortex-M55/M85 with Helium (MVE)
CONFIG_MICROLA_MVE=y
```

### Debug Options

```conf
CONFIG_DEBUG=y                       # Enable debug symbols
CONFIG_DEBUG_INFO=y                  # Include debug information
CONFIG_LOG=y                         # Enable logging
CONFIG_LOG_DEFAULT_LEVEL=3           # Set log level (0-4)
```

## Test Results

### Expected Output

When tests run successfully, you should see output like:

```
*** Booting Zephyr OS build v3.x.x ***
Running TESTSUITE microla_vector
===================================================================
START - test_default_constructor
 PASS - test_default_constructor in 0.001 seconds
START - test_variadic_constructor
 PASS - test_variadic_constructor in 0.001 seconds
...
TESTSUITE microla_vector succeeded

Running TESTSUITE microla_matrix
===================================================================
...
TESTSUITE microla_matrix succeeded

Running TESTSUITE microla_quaternion
===================================================================
...
TESTSUITE microla_quaternion succeeded

Running TESTSUITE microla_integration
===================================================================
...
TESTSUITE microla_integration succeeded

------ TESTSUITE SUMMARY START ------
SUITE PASS - 100.00% [microla_vector]: pass = 25, fail = 0, skip = 0, total = 25
SUITE PASS - 100.00% [microla_matrix]: pass = 20, fail = 0, skip = 0, total = 20
SUITE PASS - 100.00% [microla_quaternion]: pass = 18, fail = 0, skip = 0, total = 18
SUITE PASS - 100.00% [microla_integration]: pass = 15, fail = 0, skip = 0, total = 15
------ TESTSUITE SUMMARY END ------

PROJECT EXECUTION SUCCESSFUL
```

## Supported Platforms

The tests are configured to run on:

- **native_posix** - Linux/macOS host emulation (easiest for development)
- **native_sim** - Newer native simulator
- **qemu_cortex_m3** - ARM Cortex-M3 emulation
- **qemu_cortex_m0** - ARM Cortex-M0 emulation
- **qemu_cortex_a53** - ARM Cortex-A53 emulation
- Any Zephyr-supported hardware board with sufficient resources

### Minimum Hardware Requirements

- **RAM**: 32 KB minimum
- **Flash**: 64 KB minimum
- **FPU**: Recommended for performance (software fallback available)
- **C++20**: Required compiler support

## Troubleshooting

### Build Errors

**Issue**: `CMake Error: Could not find Zephyr`

```bash
# Solution: Set Zephyr environment
source <zephyr-sdk-path>/zephyr-env.sh
# Or
export ZEPHYR_BASE=<path-to-zephyr>
```

**Issue**: `undefined reference to microla functions`

```bash
# Solution: Ensure CONFIG_MICROLA=y in prj.conf
# Verify MicroLA is properly registered as a Zephyr module
```

### Runtime Errors

**Issue**: Tests fail with stack overflow

```conf
# Solution: Increase stack size in prj.conf
CONFIG_MAIN_STACK_SIZE=8192
```

**Issue**: Out of memory errors

```conf
# Solution: Increase heap size in prj.conf
CONFIG_HEAP_MEM_POOL_SIZE=32768
```

**Issue**: Floating point assertions fail
- Check that `CONFIG_FPU=y` is enabled
- Verify epsilon values are appropriate for your platform's precision
- Some low-precision platforms may need relaxed epsilon values

### Platform-Specific Issues

**native_posix**: If tests hang, check console output settings

```conf
CONFIG_UART_CONSOLE=y
CONFIG_CONSOLE=y
```

**QEMU**: If QEMU doesn't exit after tests, add to CMakeLists.txt:

```cmake
set_property(TARGET app PROPERTY PROPERTY QEMU_EXTRA_FLAGS "-nographic")
```

## Adding Tests

To add test cases:

1. **Create test function**:
   ```cpp
   ZTEST(suite_name, test_case_name)
   {
       // Your test code
       zassert_true(condition, "Error message");
   }
   ```

2. **Add to appropriate test file** or create one in `src/`

3. **Update CMakeLists.txt** if adding a source file:
   ```cmake
   target_sources(app PRIVATE
       src/main.c
       src/test_vector.cpp
       src/test_matrix.cpp
       src/test_quaternion.cpp
       src/test_integration.cpp
       src/your_new_test.cpp  # Add here
   )
   ```

4. **Rebuild and test**:
   ```bash
   west build -b native_posix -p
   west build -t run
   ```

## Continuous Integration

For CI pipelines, use Twister with JSON output:

```bash
# Run tests and generate JSON report
twister -T tests/zephyr --json-report -o test-results

# Parse results
cat test-results/twister.json | jq '.testsuites[] | select(.status != "passed")'
```

## Performance Benchmarks

While these are correctness tests, you can add performance measurements:

```cpp
ZTEST(microla_performance, test_matrix_multiply_speed)
{
    uint32_t start = k_cycle_get_32();

    // Perform operation
    Mat<float, 3, 3> result = m1 * m2;

    uint32_t end = k_cycle_get_32();
    uint32_t cycles = end - start;

    TC_PRINT("Matrix multiply took %u cycles\n", cycles);
}
```

## References

- [Zephyr Testing Documentation](https://docs.zephyrproject.org/latest/develop/test/index.html)
- [ztest API Reference](https://docs.zephyrproject.org/latest/develop/test/ztest.html)
- [MicroLA Documentation](../../README.md)
- [Twister Test Runner](https://docs.zephyrproject.org/latest/develop/test/twister.html)

## License

SPDX-License-Identifier: Apache-2.0

Copyright (c) 2026 James Baldwin
