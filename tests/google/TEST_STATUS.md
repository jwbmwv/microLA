# Test Status Report

**Date:** March 29, 2026  
**Primary Host Preset:** `host-tests`  
**Total Tests:** 530  
**Passing:** 530 (100%)  
**Failing:** 0

## Summary

MicroLA's current host-side Google Test surface is green in a clean C++20 build. The host preset validates the shipped library headers, integration scenarios, view types, numerical helpers, embedded-contract checks, and the `sensor_fusion.hpp` API surface.

The release-style CI preset extends that proof surface further:

- `ci-release-cxx20`: `531/531` passing, including `sensor_fusion_example_smoke`

## Key Covered Areas

- Linear algebra primitives: vectors, matrices, quaternions, and views
- Numerical algorithms: QR, eigenvalue paths, fast math, numerical-stability helpers
- Geometry: primitives, containment, intersection, and frustum workflows
- Embedded profile contracts: no-allocation and compile-time gating behavior
- Public sensor fusion API: orientation estimation, relative-angle extraction, observability, drift flags, and reference-pose handling

## Sensor Fusion Coverage

The dedicated sensor-fusion tests now cover:

- accelerometer-only startup rejection and tilt-only observability
- arbitrary-unit magnetometer acceptance after calibration
- directional magnetic disturbance rejection once heading is locked
- shared Mahony or EKF measurement-only fallback on gyro dropout
- calibration reset and magnetic-reference relearning
- 6-axis drift-aware heading results
- 9-axis full-3D heading recovery
- hinge-twist and swing extraction
- stale-pair and time-skew flagging, including recent-skew compensation
- policies that reject drift-enabled heading outputs
- reference-pose capture and neutralization

The maintained `examples/sensor_fusion.cpp` program is also executed in release presets through `sensor_fusion_example_smoke`.

## Validation Notes

- The host test count in this file reflects the dedicated `host-tests` preset.
- Local reruns in this workspace also passed for `embedded-tests`, `ci-release-cxx20`, `coverage-ci`, `sanitizers-ci`, `constexpr-cxx20`, and the exact CI-style `static-analysis` lane after the latest modernization cleanup.
- Windows, macOS, and Zephyr-QEMU workflows were not rerun in this Linux workspace.

For the full lane matrix, see [../TEST_COVERAGE.md](../TEST_COVERAGE.md).

