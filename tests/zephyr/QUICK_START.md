# MicroLA Zephyr Tests - Quick Reference

## Quick Commands

### Basic Testing

```bash
# Run tests on native_posix (fastest for development)
../../scripts/run_tests.sh

# Run tests on QEMU Cortex-M3
../../scripts/run_tests.sh qemu_cortex_m3

# Clean build and run
../../scripts/run_tests.sh native_posix --clean
```

### Manual Build Commands

```bash
# Build only
west build -b native_posix

# Build with pristine configuration
west build -b native_posix -p

# Run tests
west build -t run

# Clean build directory
rm -rf build
```

### Using Different Configurations

```bash
# Minimal build (smaller footprint)
west build -b qemu_cortex_m3 -- -DCONF_FILE=prj_minimal.conf

# SIMD-optimized build
west build -b qemu_cortex_m3 -- -DCONF_FILE=prj_simd.conf

# Standard build
west build -b native_posix -- -DCONF_FILE=prj.conf
```

### Twister Test Runner

```bash
# Run all tests
twister -T .

# Run on specific platform
twister -T . -p qemu_cortex_m3

# Verbose output
twister -T . -v

# Run only specific test suite
twister -T . -s libraries.microla.vector
```

## Test Suite Coverage

| Suite | Tests | Description |
|-------|-------|-------------|
| microla_vector | ~25 | Vector construction, operations, math |
| microla_matrix | ~20 | Matrix operations, transforms, inverse |
| microla_quaternion | ~18 | Quaternion rotations, SLERP |
| microla_integration | ~15 | Combined operations, real-world scenarios |

## Platform Recommendations

| Platform | Use Case | Config |
|----------|----------|--------|
| native_posix | Development, debugging | prj.conf |
| qemu_cortex_m3 | Embedded verification | prj.conf |
| qemu_cortex_a53 | ARM64/NEON testing | prj_simd.conf |
| Hardware board | Final validation | prj_simd.conf |

## Common Test Patterns

### Running Single Suite

```bash
# Modify testcase.yaml or use twister
twister -T . -s libraries.microla.vector
```

### Debugging Failed Tests

```bash
# Build with debug symbols
west build -b native_posix -- -DCONFIG_DEBUG=y -DCONFIG_DEBUG_INFO=y

# Run with GDB (native_posix)
west debug
```

### Performance Testing

```bash
# Build with optimizations
west build -b qemu_cortex_m3 -- -DCONF_FILE=prj_simd.conf

# Check binary size
size build/zephyr/zephyr.elf
```

## Expected Test Execution Time

| Platform | Time |
|----------|------|
| native_posix | ~2-5 seconds |
| qemu_cortex_m3 | ~10-20 seconds |
| Hardware | ~5-15 seconds |

## Troubleshooting Quick Fixes

| Issue | Solution |
|-------|----------|
| Build fails | `west build -p` (pristine build) |
| Tests hang | Check `CONFIG_UART_CONSOLE=y` |
| Out of memory | Increase `CONFIG_HEAP_MEM_POOL_SIZE` |
| Stack overflow | Increase `CONFIG_MAIN_STACK_SIZE` |
| Float precision | Adjust epsilon values in tests |

## CI/CD Integration

### GitHub Actions Example

```yaml
- name: Run Zephyr Tests
  run: |
    cd tests/zephyr
    west build -b native_posix -p
    west build -t run
```

### GitLab CI Example

```yaml
test:zephyr:
  script:
    - cd tests/zephyr
    - twister -T . --inline-logs
```
