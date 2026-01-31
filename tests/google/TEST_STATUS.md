# Test Status Report

**Date:** March 7, 2026  
**Primary Host Preset:** `host-tests`  
**Total Tests:** 517  
**Passing:** 517 (100%)  
**Failing:** 0

## Summary

MicroLA's current host-side Google Test surface is green in a clean C++17 build. The host preset validates the shipped library headers, integration scenarios, view types, numerical helpers, embedded-contract checks, and the new `sensor_fusion.hpp` API surface.

The release presets extend that proof surface further:

- `ci-release-cxx17`: `518/518` passing, including `sensor_fusion_example_smoke`
- `ci-release-cxx20`: `519/519` passing, including `sensor_fusion_example_smoke`

## Key Covered Areas

- Linear algebra primitives: vectors, matrices, quaternions, and views
- Numerical algorithms: QR, eigenvalue paths, fast math, numerical-stability helpers
- Geometry: primitives, containment, intersection, and frustum workflows
- Embedded profile contracts: no-allocation and compile-time gating behavior
- Public sensor fusion API: orientation estimation, relative-angle extraction, observability, drift flags, and reference-pose handling

## Sensor Fusion Coverage

The dedicated sensor-fusion tests now cover:

- accelerometer-only startup rejection and tilt-only observability
- 6-axis drift-aware heading results
- 9-axis full-3D heading recovery
- hinge-twist and swing extraction
- stale-pair and time-skew flagging
- policies that reject drift-enabled heading outputs
- reference-pose capture and neutralization

The maintained `examples/sensor_fusion.cpp` program is also executed in release presets through `sensor_fusion_example_smoke`.

## Validation Notes

- The host test count in this file reflects the dedicated `host-tests` preset.
- Additional CI-equivalent lanes also pass locally: embedded, coverage, sanitizers, constexpr C++17/C++20, format, static analysis, and PlantUML validation.
- Windows, macOS, and Zephyr-QEMU workflows were not rerun in this Linux workspace.

For the full lane matrix, see [../TEST_COVERAGE.md](../TEST_COVERAGE.md).

