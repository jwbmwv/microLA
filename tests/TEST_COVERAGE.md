# Test Coverage

**Date:** March 29, 2026  
**Baseline:** C++20 minimum, Linux local migration validation

## Current Validation Summary

| Lane | Result | Notes |
|------|--------|-------|
| `format-check` | Pass | `clang-format-18 --dry-run -Werror` over `include/`, `tests/`, `examples/`, and `benchmarks/` |
| `host-tests` | `530/530` | Host Google Tests plus embedded-contract tests |
| `embedded-tests` | `530/530` | Host-side tests built with `MICROLA_EMBEDDED` |
| `ci-release-cxx20` | `531/531` | Includes `sensor_fusion_example_smoke` |
| `coverage-ci` | `530/530` | Coverage-instrumented tests pass |
| `sanitizers-ci` | `530/530` | ASan + UBSan lane passes |
| `constexpr-cxx20` | `530/530` | Dedicated C++20 constexpr validation |
| `static-analysis` | Pass | Exact CI-style `clang-tidy` header gate, `examples/*.cpp` gate, and `cppcheck --force` all completed cleanly after the C++20 modernization follow-up |
| `diagram-regeneration` | Pass | `scripts/generate_diagrams.sh` regenerated PNG, SVG, and PDF assets successfully |

## Coverage Highlights

- `sensor_fusion.hpp` now has direct automated host coverage through `tests/google/test_sensor_fusion.cpp`.
- The sensor-fusion coverage now also includes arbitrary-unit magnetometer acceptance, directional magnetic disturbance rejection, shared Mahony or EKF gyro-dropout fallback, calibration-reset behavior, and recent-skew compensation.
- The maintained `examples/sensor_fusion.cpp` program is now exercised by `sensor_fusion_example_smoke` in the release preset, so the documented fusion flow is not compile-only.
- Embedded allocation-contract coverage remains explicit through the dedicated embedded-contract suite.

## Local Limits

- The Windows and macOS matrix jobs were not run locally in this Linux workspace.
- The Zephyr QEMU workflow was not run locally because it requires the external Zephyr workspace and SDK setup performed by CI.
- Windows, macOS, and Zephyr-QEMU remain the only CI surfaces not rerun in this Linux workspace.
- The exact CI-style `static-analysis` lane was rerun locally after the modernization cleanup and now passes in this Linux workspace.

## Recommended Canonical References

- Use this file for the current lane-by-lane status.
- Use `tests/google/TEST_STATUS.md` for the host-test surface summary.
- Use `tests/TEST_COMPLETENESS_REPORT.md` for a higher-level assessment of what is and is not covered.
