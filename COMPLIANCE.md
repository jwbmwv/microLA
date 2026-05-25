# Compliance and Static Analysis Baseline

This document describes the project's baseline static-analysis checks and the process for exceptions and stronger compliance claims.

## Baseline rule sets (automated)

- **C++ Core Guidelines** via `clang-tidy` checks: `cppcoreguidelines-*`
- **Bug-prone checks** via `clang-tidy`: `bugprone-*`
- **Modernization** via `clang-tidy`: `modernize-*`
- **Performance hints** via `clang-tidy`: `performance-*`
- **Style/formatting** via `clang-format` (style chosen: `LLVM`)
- **Static checks** via `cppcheck` with `warning,performance,portability` and `--force`

These are enforced in CI and are the expected minimum for contributions.

## CI enforcement

- `clang-tidy` runs with `-checks=cppcoreguidelines-*,bugprone-*,modernize-*,performance-*` and treats warnings as failures.
- `cppcheck` runs with `--force --enable=warning,performance,portability` and exits non-zero on findings.
- `clang-format` is enforced via formatting checks and the `scripts/format.sh --check` helper.

See `.github/workflows/ci.yml` (the `static-analysis` and `format-check` jobs) for the exact CI job configuration.

## Exceptions and deviations

If a rule must be suppressed or deviated from, contributors must:

1. Open a pull request describing the reason for the deviation.
2. Add a short code comment near the deviation referencing the PR number and rationale.
3. Add an entry to `COMPLIANCE.md` under "Deviations" with a one-line justification and link to the PR.

Example deviation entry:

```
2026-02-08: Allow use of raw pointer in `src/legacy_adapter.cpp` for performance reasons. See PR #123.
```

## Stronger claims (MISRA / AUTOSAR / certified tools)

If a customer requires formal compliance (e.g., MISRA C++), this repository will:

- Run a commercial MISRA/AUTOSAR checker (PC-Lint, Parasoft, LDRA, etc.) in CI or as a gated step.
- Maintain a formal deviation log with justification and mitigation.

These are out-of-band from the automated baseline and typically require licensing.

## Getting started (local)

Install tools and run locally:

```bash
sudo apt-get install clang-tidy cppcheck clang-format-18
cmake --preset static-analysis
BUILD_DIR=build/static-analysis \
TIDY_INCLUDE_REGEX='.*/examples/.*[.](cpp|c)$' \
./scripts/tidy.sh --config-file="$PWD/.clang-tidy" --header-filter='^$' --warnings-as-errors='*'
cppcheck --force --enable=warning,performance,portability --std=c++20 --suppress=missingIncludeSystem . --error-exitcode=1
./scripts/format.sh --check
```

## Contact

If you have questions about the compliance process, open an issue labeled `ci` or `compliance`.
