# Test Coverage

**Date:** March 7, 2026  
**Baseline:** C++17 minimum, Linux local CI-equivalent validation

## Current Validation Summary

| Lane | Result | Notes |
|------|--------|-------|
| `format-check` | Pass | `clang-format-18 --dry-run -Werror` over `include/`, `tests/`, `examples/`, `benchmarks/` |
| `host-tests` | `517/517` | Host Google Tests plus embedded-contract tests |
| `embedded-tests` | `517/517` | Host-side tests built with `MICROLA_EMBEDDED` |
| `ci-release-cxx17` | `518/518` | Includes `sensor_fusion_example_smoke` |
| `ci-release-cxx20` | `519/519` | Includes `sensor_fusion_example_smoke` and C++20-only coverage |
| `coverage-ci` | `517/517` | Coverage-instrumented tests pass |
| `sanitizers-ci` | `517/517` | ASan + UBSan lane passes |
| `constexpr-cxx17` | `517/517` | C++17 constexpr validation |
| `constexpr-cxx20` | `518/518` | C++20 constexpr validation |
| `embedded-examples` | Build passes | Embedded examples compile cleanly |
| `static-analysis` | Pass | `clang-tidy` umbrella-header gate, example TUs, and `cppcheck --force` |
| `plantuml-validation` | Pass | CI-equivalent design validation passes; local rerun also validated example diagrams |

## Coverage Highlights

- `sensor_fusion.hpp` now has direct automated host coverage through `tests/google/test_sensor_fusion.cpp`.
- The sensor-fusion coverage includes invalid-startup handling, magnetometer rejection fallback, drift-aware heading output, hinge twist, swing extraction, reference-pose capture, and stale/skew flagging.
- The maintained `examples/sensor_fusion.cpp` program is now exercised by `sensor_fusion_example_smoke` in the release presets, so the documented fusion flow is not compile-only.
- Embedded allocation-contract coverage remains explicit through the dedicated embedded-contract suite.

## Local Limits

- The Windows and macOS matrix jobs were not run locally in this Linux workspace.
- The Zephyr QEMU workflow was not run locally because it requires the external Zephyr workspace and SDK setup performed by CI.
- `lcov` is not installed in this environment, so the coverage preset was executed but the local HTML or summary coverage report was not regenerated here.

## Recommended Canonical References

- Use this file for the current lane-by-lane status.
- Use `tests/google/TEST_STATUS.md` for the host-test surface summary.
- Use `tests/TEST_COMPLETENESS_REPORT.md` for a higher-level assessment of what is and is not covered.
