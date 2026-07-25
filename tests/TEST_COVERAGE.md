# Test Coverage

**Baseline:** C++20 minimum, Linux validation guidance

## Current Validation Summary

| Lane | Result | Notes |
|------|--------|-------|
| `format-check` | Pass | `clang-format-18 --dry-run -Werror` over `include/`, `tests/`, `examples/`, and `benchmarks/` |
| `host-tests` | Run `ctest --preset host-tests` | Host Google Tests plus embedded-contract tests |
| `embedded-tests` | Run `ctest --preset embedded-tests` | Host-side tests built with `MICROLA_EMBEDDED` |
| `ci-release-cxx20` | Run `ctest --preset ci-release-cxx20` | Includes `sensor_fusion_example_smoke` |
| `coverage-ci` | Run `ctest --preset coverage-ci` | Coverage-instrumented tests |
| `sanitizers-ci` | Run `ctest --preset sanitizers-ci` | ASan + UBSan + alignment validation |
| `constexpr-cxx20` | Run `ctest --preset constexpr-cxx20` | Dedicated C++20 constexpr validation |
| `static-analysis` | Pass | Exact CI-style `clang-tidy` header gate, `examples/*.cpp` gate, and `cppcheck --force` all completed cleanly after the C++20 modernization follow-up |
| `diagram-regeneration` | Pass | `scripts/generate_diagrams.sh` regenerated PNG, SVG, and PDF assets successfully |

## Dated Validation Evidence

**Recorded:** 2026-07-25 on Linux. This is a release-engineering snapshot, not a
claim that every target has executed on physical hardware.

| Surface | Evidence | Result | Boundary |
|---|---|---|---|
| Host release | `ctest --preset host-tests` | 553/553 passed | Native Linux execution only |
| Embedded policy | `ctest --preset embedded-tests` | 553/553 passed | Executes on Linux with `MICROLA_EMBEDDED`; it does not validate target ABI or timing |
| Sanitizers | `ctest --preset sanitizers-ci` with ASan, UBSan, and alignment checks | 553/553 passed | Clang/Linux only |
| Coverage instrumentation | `ctest --preset coverage-ci` | 553/553 passed | Measures host code paths; it is not a target coverage result |
| Sensor-fusion replay | `ctest --test-dir build/host-tests -R SensorFusionTraceReplay` | 3/3 passed | Deterministic fixtures cover fault and timing transitions, not deployment sensor traces |
| Documentation snippets | `python3 scripts/verify_snippets.py --require-compiled` | 2 standalone snippets compiled; 71 fragments explicitly skipped | Fragments and placeholders are reported rather than treated as compiled programs |
| Arm compile-only resource profiles | Cortex-M0+, M3, M4F, and M7F presets | Passed locally; object-size and `.su` reports emitted | No board linker script, peripheral integration, or hardware execution |
| AArch64 and RV64GC compile-only resource profiles | CI `arm-target-compile` matrix | CI-provisioned; not rerun locally | No local toolchain or runtime execution claim |
| Zephyr QEMU | CI `zephyr-qemu` job | CI-provisioned for Cortex-M0, Cortex-M3, and native simulation | External Zephyr workspace and SDK required |
| Zephyr Renode nRF52840 | CI `zephyr-renode-nrf52840` job | CI-provisioned Cortex-M4F SoC runtime lane | No local Renode installation or physical-device claim |

## Known Validation Gaps

- Windows and macOS execution remain CI-only and were not rerun in this Linux workspace.
- QEMU and Renode validate software and selected SoC models, not a board's wiring, clocks, sensors, interrupt load, flash/RAM layout, or worst-case execution time.
- AArch64 and RV64GC builds are compile-only CI checks; they do not provide runtime or hardware validation.
- The sensor-fusion replay suite is intentionally deterministic. Release qualification still requires recorded traces from the deployed sensor, mounting geometry, and timing source.
- Markdown fragments without a complete standalone program are visible as skipped in the snippet report. Promote high-risk fragments to maintained standalone examples when they become a support contract.

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
