# Quick Start Guide

## Repository Structure

```
microla/
├── README.md              # Main documentation
├── LICENSE                # Apache License 2.0 with SPDX identifier
├── VERSION                # Version number (0.0.1)
├── CMakeLists.txt         # Main CMake configuration
├── .gitignore             # Git ignore rules
├── .ai-generation-prompt.md  # AI regeneration prompt
│
├── include/microla/     # Public headers
│   ├── compiler_features.hpp  # C++ feature detection macros (MICROLA_CONSTEXPR, etc.)
│   ├── microla.hpp    # Convenience header (includes all components)
│   ├── constants.hpp      # Mathematical constants (independent)
│   ├── vector.hpp         # Generic Vec<T,N> template
│   ├── vector_view.hpp    # Non-owning vector view
│   ├── matrix.hpp         # Generic Mat<T,R,C> and SquareMat<T,N>
│   ├── matrix_view.hpp    # Non-owning matrix view
│   ├── quaternion.hpp     # Quaternion class
│   ├── geometry.hpp       # AABB, Sphere, Ray, Plane, Frustum
│   ├── kalman.hpp         # Kalman filter
│   ├── extended_kalman.hpp # Extended Kalman filter (EKF)
│   ├── fast_math.hpp      # Fast approximate math functions
│   ├── safe_math.hpp      # Numerically safe math operations
│   ├── numerical_stability.hpp # Kahan summation, condition numbers
│   ├── high_precision.hpp # High-precision arithmetic
│   ├── resource_checks.hpp # Compile-time resource limit checks
│   ├── simd_helpers.hpp   # SIMD abstraction (NEON/SSE/CMSIS-DSP)
│   └── version.hpp        # Version API
│
├── docs/                  # Documentation
│   └── API_Documentation.md
│
├── examples/              # Example programs
│   ├── basic_usage.cpp
│   └── CMakeLists.txt
│
├── tests/                 # Unit tests
│   ├── README.md
│   ├── CMakeLists.txt
│   ├── google/            # Google Test suite
│   └── zephyr/            # Zephyr ztest suite
│
├── zephyr/               # Zephyr RTOS integration
│   ├── module.yml        # Zephyr module metadata
│   ├── CMakeLists.txt    # Zephyr build configuration
│   ├── Kconfig           # Configuration options (NEON/CMSIS)
│   └── README.md         # Zephyr usage guide
│
└── cmake/                # CMake modules
    └── microla-config.cmake.in
```

## Usage Methods

### Method 1: Git Submodule (Generic Projects)

```bash
# Add submodule
git submodule add https://github.com/jwbmwv/microla.git external/microla

# In your CMakeLists.txt
add_subdirectory(external/microla)
target_link_libraries(your_app PUBLIC microla)
```

### Method 2: Zephyr Module

Add to `west.yml`:

```yaml
manifest:
  projects:
    - name: microla
      url: https://github.com/jwbmwv/microla.git
      path: modules/lib/microla
```

Enable in `prj.conf`:

```
CONFIG_MICROLA_LINEAR=y
```

### Method 3: CMake FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
  microla
  GIT_REPOSITORY https://github.com/jwbmwv/microla.git
  GIT_TAG main
)
FetchContent_MakeAvailable(microla)
target_link_libraries(your_app PUBLIC microla)
```

### Method 4: Direct Copy

Just copy `include/microla/` to your project's include path.

### Method 5: IAR Embedded Workbench for ARM

#### Option A: Add to IAR Project Directly

1. In IAR Embedded Workbench, right-click your project → **Options**
2. Navigate to **C/C++ Compiler** → **Preprocessor** → **Additional include directories**
3. Add the path to `microla/include`
4. In your source files:
   ```cpp
   // Option 1: Include everything

   #include <microla/microla.hpp>

   #include <microla/quaternion.hpp>

   // Option 2: Modular includes (faster compilation)

   #include <microla/vector.hpp>      // Base vector template

   #include <microla/vec3D.hpp>       // 3D type aliases

   #include <microla/matrix3D.hpp>    // 3D transformations

   using namespace microla;
   ```

#### Option B: Use CMake with IAR

```bash
# Configure with IAR toolchain
cmake -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=iar-toolchain.cmake ..
cmake --build .
```

Create `iar-toolchain.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Path to IAR installation
set(IAR_ARM_ROOT "C:/Program Files/IAR Systems/Embedded Workbench 9.x/arm")

set(CMAKE_C_COMPILER "${IAR_ARM_ROOT}/bin/iccarm.exe")
set(CMAKE_CXX_COMPILER "${IAR_ARM_ROOT}/bin/iccarm.exe")
set(CMAKE_ASM_COMPILER "${IAR_ARM_ROOT}/bin/iasmarm.exe")
set(CMAKE_AR "${IAR_ARM_ROOT}/bin/iarchive.exe")

set(CMAKE_C_COMPILER_ID IAR)
set(CMAKE_CXX_COMPILER_ID IAR)

# Prevent CMake from testing the compiler
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
```

#### IAR-Specific Configuration

**Compiler Settings:**
- **Language**: C++20 or later (Options → C/C++ Compiler → Language → C++ → C++20)
- **Optimization**: High for performance (Options → C/C++ Compiler → Optimizations)
- **NEON**: Enable VFPv4_sp for NEON support (Options → C/C++ Compiler → Code)

**Preprocessor Defines:**

```
CONFIG_MICROLA_NEON    // For NEON optimization
```

## Building Examples

```bash
cmake --preset debug
cmake --build --preset debug
./build/debug/examples/basic_usage
```

## Enabling SIMD Optimizations

### ARM NEON (Cortex-A, ARM64)

```cmake
target_compile_definitions(your_target PRIVATE CONFIG_MICROLA_NEON)
```

### CMSIS-DSP (Cortex-M)

```cmake
target_compile_definitions(your_target PRIVATE CONFIG_MICROLA_CMSIS)
target_link_libraries(your_target PRIVATE arm_cortexM4lf_math)  # Or appropriate CMSIS variant
```

### RISC-V V Extensions

```cmake
target_compile_definitions(your_target PRIVATE CONFIG_MICROLA_RISCV)
target_compile_options(your_target PRIVATE -march=rv64gcv)  # Enable V extension
```

### Auto-detect (All Platforms)

```cmake
target_compile_definitions(your_target PRIVATE MICROLA_AUTODETECT_SIMD)
```

### ARM NEON (Cortex-A, ARM64, Apple Silicon)

```cmake
# Option 1: Use the shipped preset
cmake --preset neon
cmake --build --preset neon

# Option 2: In your CMakeLists.txt
target_compile_definitions(your_app PRIVATE CONFIG_MICROLA_NEON)
```

### CMSIS-DSP (Cortex-M, CMake Options)

```cmake
# Option 1: Use the shipped preset
cmake --preset cmsis
cmake --build --preset cmsis

# Option 2: In your CMakeLists.txt
target_compile_definitions(your_app PRIVATE CONFIG_MICROLA_CMSIS)
target_link_libraries(your_app PRIVATE CMSIS::DSP)
```

### Zephyr Configuration

```ini
# In prj.conf
CONFIG_MICROLA_LINEAR=y
CONFIG_MICROLA_NEON=y   # For Cortex-A/ARM64
CONFIG_MICROLA_CMSIS=y  # For Cortex-M
CONFIG_MICROLA_MVE=y    # For Cortex-M55/M85 (Helium)
```

## Testing

```bash
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests
```

### Embedded-style build (no exceptions, no dynamic allocation)

To validate embedded constraints locally, use the dedicated preset that enables `MICROLA_EMBEDDED` and the size-focused defaults used by CI:

```bash
cmake --preset embedded-tests
cmake --build --preset embedded-tests
ctest --preset embedded-tests
```

The repository also includes a GitHub Actions workflow `.github/workflows/embedded-build.yml` which performs the same steps on push and pull requests to `main`.

## First Program

```cpp
#include <microla/microla.hpp>   // Includes all vector/matrix components
#include <microla/quaternion.hpp>
#include <microla/version.hpp>
#include <iostream>

// Or use modular includes:
// #include <microla/vector.hpp>
// #include <microla/vec3D.hpp>
// #include <microla/matrix3D.hpp>

using namespace microla;

int main() {
    // Check library version
    std::cout << "MicroLA " << get_version_string() << "\n";

    // Vector operations
    Vec<float, 3> v1(1.0f, 0.0f, 0.0f);
    Vec<float, 3> v2(0.0f, 1.0f, 0.0f);
    Vec<float, 3> cross = v1.cross(v2);

    std::cout << "Cross: (" << cross[0] << ", "
              << cross[1] << ", " << cross[2] << ")\n";

    // Quaternion rotation
    Quaternion<float> q(Vec<float, 3>(0, 0, 1), 1.57f);
    Vec<float, 3> rotated = q.rotate(v1);

    std::cout << "Rotated: (" << rotated[0] << ", "
              << rotated[1] << ", " << rotated[2] << ")\n";

    return 0;
}
```

## Documentation

- [README.md](README.md) - Overview and quick start
- [docs/API_Documentation.md](docs/API_Documentation.md) - Complete API reference
- [zephyr/README.md](zephyr/README.md) - Zephyr integration guide
- [tests/README.md](tests/README.md) - Testing guide

## Requirements

- C++20 or later
- CMake 3.13.1+ (for building)
- Optional: ARM NEON (for Cortex-A/ARM64 optimization)
- Optional: CMSIS-DSP (for Cortex-M optimization)
- Optional: Zephyr RTOS 3.0+ (for Zephyr integration)

## Version

Current version: 0.0.1 (see [VERSION](VERSION))

## License

Apache License 2.0 - see [LICENSE](LICENSE)
