# MicroLA Release Contract

This document defines the support boundary for MicroLA `0.0.3`. The canonical version is the repository `VERSION` file; CMake, Conan, and vcpkg package metadata must match it.

## Language and Configuration

- The public API requires C++20. C++23 and C++26 facilities are optional enhancements, not a different support baseline.
- The package CMake configuration has a low minimum CMake requirement; the checked-in preset workflow requires CMake 3.23 or newer.
- Fixed-size `Vec`, `Mat`, `Quaternion`, and sensor-fusion estimators store their normal working state inline. Core update and compute paths do not require dynamic allocation.
- Some optional APIs intentionally provide allocating overloads. Embedded builds define `MICROLA_NO_DYNAMIC_ALLOC`; applications with a process-wide allocation ban must enforce that policy in their own runtime as well.
- The CMake embedded profile propagates `-fno-exceptions` and `-fno-rtti` for GCC/Clang consumers and defines the matching MicroLA policy macros. Direct-header consumers must configure their compiler flags themselves when they require those restrictions.

## Ownership, Timing, and ISR Use

- Value types are safe for concurrent read-only access. Mutable algorithms, including `OrientationEstimator` and `RelativeAngleEstimator`, are not internally synchronized. Serialize updates and computations or use one estimator instance per execution context.
- Do not update an estimator concurrently from an ISR and a task without application-level ownership or synchronization. Keep ISR work bounded and defer policy changes, calibration changes, and result handling when application timing requires it.
- Sensor-fusion timestamps are seconds. A negative timestamp delta is rejected without mutating orientation state. Use `rebase_timestamp()` to choose a new local origin while preserving estimator state; rebase both sides of a relative estimator to a common origin.
- Changing calibration resets the affected estimator and clears any captured relative reference pose.

## Validation Scope

| Configuration | Evidence | Limit |
|---|---|---|
| Host C++20 | Release, sanitizer, coverage, constexpr, static-analysis, and snippet gates | Host execution does not prove target ABI, timing, or code size |
| Embedded host profile | Full tests with `MICROLA_EMBEDDED` policy enabled | Still executes on the host |
| Zephyr | CI Twister runs on `qemu_cortex_m0`, `qemu_cortex_m3`, and `native_sim` | QEMU is not physical-device validation |
| Cortex-M0+, M3, M4F, M7F | Arm GNU compile-only resource smoke matrix | No board linker, startup code, peripheral integration, or hardware execution |
| AArch64 and RV64GC | CI-provisioned GNU compile-only resource smoke matrix | No local hardware execution claim |

The deterministic trace-replay suite exercises stationary and yaw motion, magnetometer innovation rejection, gyro saturation, freefall, high-g acceleration, reordering, timestamp rebasing, paired sample skew, and calibration/reference invalidation. It is a regression guard, not a substitute for recorded data from the deployed sensor and mounting geometry.

## Resource and Backend Selection

Mahony is the default lower-footprint orientation backend. MEKF is opt-in and carries covariance, matrix, and stack costs that must be reviewed on the selected compiler and MCU.

Run a target preset to generate the static report:

```bash
cmake --preset arm-cortex-m4f-compile
cmake --build --preset arm-cortex-m4f-compile
cat build/arm-cortex-m4f-compile/microla-target-resource-report.txt
```

The report contains object section sizes plus per-function GCC stack usage. It is not a whole-image flash/RAM budget or a WCET measurement. Establish application budgets after linking with the board linker script and measuring on clocked hardware under representative interrupt load.

For persistent estimator-state planning, build and run the ABI-specific size probe, then pass the expected instance counts to the calculator:

```bash
cmake --preset resource-plan
cmake --build --preset resource-plan
./build/resource-plan/microla_resource_plan > build/resource-plan/type-sizes.json
python3 scripts/estimate_sensor_fusion_footprint.py \
	build/resource-plan/type-sizes.json \
	--mahony-orientations 2 \
	--mekf-relative-pairs 1 \
	--stack-report build/arm-cortex-m4f-compile/CMakeFiles/microla_target_smoke.dir/tools/target_smoke.cpp.su
```

The calculator totals persistent estimator objects only. It reports the largest individual compiler frame from supplied `.su` files but cannot derive full call-path, ISR-nesting, linked-image, or peripheral memory bounds.

## SIMD Scope

- Scalar implementations are the portable baseline.
- NEON is for NEON-capable Arm application processors; 32-bit Arm enables the relevant compiler flag, while AArch64 relies on its target architecture support.
- CMSIS-DSP is optional and requires a compatible external CMSIS-DSP integration supplied by the application.
- MVE configuration support targets Helium-capable Cortex-M devices; verify generated kernels and timing on the chosen device.
- RISC-V compile coverage verifies the scalar-compatible build. RISC-V Vector acceleration is not a release performance promise.

## Release Evidence

Benchmark CI builds the pinned Google Benchmark 1.8.3 revision in Release mode and retains JSON plus environment metadata. Use `benchmark-ci` for comparable CI measurements and `benchmark` only for developer-native measurements. Historical values in [PERFORMANCE.md](../PERFORMANCE.md) are not release claims until reproduced.

Before shipping, run the appropriate CMake preset and test preset for the release target, review target resource artifacts, and perform board-level functional, timing, memory, and fault-injection validation for the intended sensor, compiler, and runtime configuration.
