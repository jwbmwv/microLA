# Sample Zephyr Project Configuration for MicroLA

This directory contains Zephyr RTOS integration files for MicroLA.

## Files

- **module.yml**: Zephyr module metadata
- **CMakeLists.txt**: Zephyr-specific build configuration
- **Kconfig**: Configuration options for MicroLA

## Using MicroLA in Zephyr

### 1. Add as a Zephyr Module

In your application's `west.yml`:

```yaml
manifest:
  projects:
    - name: microla
      url: https://github.com/microla/microla.git
      revision: main
      path: modules/lib/microla
```

Then run:
```bash
west update
```

### 2. Enable in prj.conf

```conf
# Enable MicroLA
CONFIG_MICROLA=y

# Optional: Enable ARM NEON optimizations (Cortex-A/ARM64 only)
CONFIG_MICROLA_NEON=y

# Optional: Enable CMSIS-DSP optimizations (Cortex-M only)
CONFIG_MICROLA_CMSIS=y
CONFIG_CMSIS_DSP=y

# Optional: Enable ARM MVE (Helium) optimizations (Cortex-M55/M85)
CONFIG_MICROLA_MVE=y
```

### 3. Use in Your Application

```c
// In CMakeLists.txt
target_link_libraries(app PUBLIC microla)

// In your source code
#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <microla/version.hpp>

using namespace microla;

void main(void) {
    printk("MicroLA %s\n", get_version_string().c_str());
    
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    float len = v.length();
    printk("Vector length: %f\n", len);
}
```

## Configuration Options

### CONFIG_MICROLA
Enable the MicroLA library. This adds the include path and makes the library available to your application.

### CONFIG_MICROLA_NEON
Enable ARM NEON SIMD optimizations. Requires:
- ARM Cortex-A processor or ARM64/AArch64
- Automatically adds `-mfpu=neon` compiler flag

When enabled, vector and quaternion operations for `Vec<float, 2/3/4>` and `Quaternion<float>` use ARM NEON intrinsics for 2-4x performance improvement.

### CONFIG_MICROLA_CMSIS
Enable CMSIS-DSP hardware acceleration. Requires:
- ARM Cortex-M processor  
- CONFIG_CMSIS_DSP=y

When enabled, vector, matrix, and quaternion operations automatically use optimized CMSIS-DSP functions for `float` types.

### CONFIG_MICROLA_MVE
Enable ARM MVE (Helium) optimizations. Requires:
- ARM Cortex-M55 or Cortex-M85 processor.
- Automatically adds `-march=armv8.1-m.main+mve` compiler flag.

When enabled, MicroLA can compile MVE-specific paths where available. (MVE kernels are currently reserved for future optimization work.)

### CONFIG_MICROLA_DEBUG
Enable debug assertions and bounds checking. Adds runtime checks for development and testing. Disable for production builds.

## Example Application

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(microla_demo)

target_sources(app PRIVATE src/main.cpp)
target_link_libraries(app PUBLIC microla)
```

```c++
// src/main.cpp
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <microla/version.hpp>

using namespace microla;

void main(void)
{
    printk("MicroLA %s Demo\n", get_version_string().c_str());
    
    // Vector operations
    Vec<float, 3> a(1.0f, 0.0f, 0.0f);
    Vec<float, 3> b(0.0f, 1.0f, 0.0f);
    Vec<float, 3> c = a.cross(b);
    
    printk("Cross product: (%f, %f, %f)\n", c[0], c[1], c[2]);
    
    // Quaternion rotation
    Quaternion<float> q(Vec<float, 3>(0, 0, 1), 1.57f);
    Vec<float, 3> rotated = q.rotate(a);
    
    printk("Rotated: (%f, %f, %f)\n", rotated[0], rotated[1], rotated[2]);
}
```

## Performance on Zephyr

MicroLA is optimized for embedded systems:
- Zero dynamic allocation
- POD types compatible with Zephyr
- Optional CMSIS-DSP acceleration on ARM
- Minimal flash/RAM footprint

Typical footprint (ARM Cortex-M4F, -Os):
- Vec operations: ~100-500 bytes
- Matrix operations: ~500-2000 bytes
- Quaternion operations: ~300-1000 bytes

With CMSIS-DSP, operations can be 2-5× faster while using the same or less code space.
