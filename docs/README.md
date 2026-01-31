# MicroLA Documentation

Comprehensive documentation for MicroLA - a header-only C++11 linear algebra library.

## Quick Links

- [Main README](../README.md) - Overview and quick start
- [Quick Reference](../QUICK_REFERENCE.md) - API reference card
- [Quickstart Guide](../QUICKSTART.md) - Getting started tutorial
- [Changelog](../CHANGELOG.md) - Version history

## Detailed Documentation

### Core Documentation
- **[API_Documentation.md](API_Documentation.md)** - Complete API reference
- **[COOKBOOK.md](COOKBOOK.md)** - Common use cases and examples
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions

### Advanced Topics
- **[SIMD_Optimizations.md](SIMD_Optimizations.md)** - NEON and CMSIS-DSP optimization details
- **[Cpp_Standard_Optimizations.md](Cpp_Standard_Optimizations.md)** - C++11-C++26 feature usage

### Integration Guides
- **[Performance](../PERFORMANCE.md)** - Benchmarks and optimization tips
- **[Contributing](../CONTRIBUTING.md)** - Development guidelines
- **[Architecture](../ARCHITECTURE.md)** - Design decisions and rationale

## Examples

See the [examples/](../examples/) directory for:
- `basic_usage.cpp` - Comprehensive API demonstrations
- `constexpr_rotations.cpp` - Compile-time rotation matrices
- `sensor_fusion.cpp` - IMU sensor fusion example
- `robotics_kinematics.cpp` - Robot arm kinematics
- `graphics_pipeline.cpp` - 3D graphics transformations
- `advanced_features_demo.cpp` - Block operations and decompositions

## Design Diagrams

PlantUML diagrams in [design/](../design/) directory visualize:
- Class hierarchy and relationships
- SIMD optimization strategy
- C++ standard feature progression
- Memory layout and alignment
- Module structure and dependencies
