# MicroLA Documentation Index

**Version:** 0.0.3
**Release Date:** May 31, 2026

## 📚 Documentation Structure

### Root Level Documentation

| File | Description |
|------|-------------|
| [README.md](README.md) | Main project overview, features, and quick start |
| [QUICKSTART.md](QUICKSTART.md) | Step-by-step getting started guide |
| [QUICK_REFERENCE.md](QUICK_REFERENCE.md) | API reference card for quick lookup |
| [CHANGELOG.md](CHANGELOG.md) | Version history and changes |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Development guidelines and contribution process |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Design decisions and architectural rationale |
| [PERFORMANCE.md](PERFORMANCE.md) | Benchmarks and optimization tips |
| [LICENSE](LICENSE) | Apache License 2.0 |
| [NOTICE](NOTICE) | AI generation disclosure and IP notices |

### Detailed Documentation (`docs/`)

| File | Purpose |
|------|---------|
| [API_Documentation.md](docs/API_Documentation.md) | Complete API reference for all classes |
| [COOKBOOK.md](docs/COOKBOOK.md) | Common use cases and code examples |
| [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Solutions to common issues |
| [SIMD_Optimizations.md](docs/SIMD_Optimizations.md) | NEON and CMSIS-DSP optimization details |
| [Cpp_Standard_Optimizations.md](docs/Cpp_Standard_Optimizations.md) | C++17-C++26 feature usage |
| [SENSOR_FUSION.md](docs/SENSOR_FUSION.md) | Relative sensor fusion, observability, and tuning knobs |
| [RELEASE_CONTRACT.md](docs/RELEASE_CONTRACT.md) | Supported configurations, resource checks, and release validation scope |

### Design Diagrams (`design/`)

All diagrams use PlantUML format and document system architecture:

| Diagram | Content |
|---------|---------|
| [class_hierarchy.puml](design/class_hierarchy.puml) | Classes, relationships, and interfaces |
| [simd_optimization.puml](design/simd_optimization.puml) | Three-tier SIMD strategy (NEON/CMSIS/Generic) |
| [cpp_standard_features.puml](design/cpp_standard_features.puml) | C++17-C++26 progressive enhancement |
| [memory_layout.puml](design/memory_layout.puml) | Memory organization and alignment |
| [module_structure.puml](design/module_structure.puml) | Header dependencies and build integration |

**Generate diagrams:**

```bash
# Linux/macOS
bash scripts/generate_diagrams.sh

# Windows
scripts/generate_diagrams.bat
```

### Examples (`examples/`)

| Example | Features Demonstrated |
|---------|----------------------|
| [basic_usage.cpp](examples/basic_usage.cpp) | Comprehensive API demonstration |
| [constexpr_rotations.cpp](examples/constexpr_rotations.cpp) | Compile-time rotation matrices |
| [sensor_fusion.cpp](examples/sensor_fusion.cpp) | Mixed 9-axis plus 6-axis relative-angle estimation with drift diagnostics |
| [robotics_kinematics.cpp](examples/robotics_kinematics.cpp) | Robot arm forward/inverse kinematics |
| [graphics_pipeline.cpp](examples/graphics_pipeline.cpp) | 3D graphics transformation pipeline |
| [advanced_features_demo.cpp](examples/advanced_features_demo.cpp) | Block operations and decompositions |

**Build examples:**

```bash
cmake --preset debug
cmake --build --preset debug
```

### Benchmarks (`benchmarks/`)

Performance measurement programs for:
- Vector operations (SIMD vs scalar)
- Matrix multiplication (2×2, 3×3, 4×4)
- Quaternion operations (multiply, SLERP)
- Constexpr evaluation overhead

**Build benchmarks:**

```bash
cmake --preset benchmark
cmake --build --preset benchmark
```

## 🎯 Quick Navigation

### For First-Time Users

1. Start with [README.md](README.md) for overview
2. Follow [QUICKSTART.md](QUICKSTART.md) for integration
3. Browse [examples/basic_usage.cpp](examples/basic_usage.cpp) for patterns

### For API Reference

- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Quick lookup
- [docs/API_Documentation.md](docs/API_Documentation.md) - Complete reference

### For Optimization

- [PERFORMANCE.md](PERFORMANCE.md) - Benchmark results
- [docs/SIMD_Optimizations.md](docs/SIMD_Optimizations.md) - SIMD details
- [design/simd_optimization.puml](design/simd_optimization.puml) - Visual guide

### For Contributors

- [CONTRIBUTING.md](CONTRIBUTING.md) - Development guidelines
- [ARCHITECTURE.md](ARCHITECTURE.md) - Design philosophy
- [design/](design/) - Architecture diagrams

## 🔧 Tools and Configuration

### PlantUML Diagram Generation

- **Outputs:** PNG, SVG, PDF
- **Scripts:** [scripts/generate_diagrams.sh](scripts/generate_diagrams.sh), [scripts/generate_diagrams.bat](scripts/generate_diagrams.bat)
- **Coverage:** all `.puml` files under `design/` and `examples/`

### Build System

- **CMake:** 3.13+ for basic package configuration; 3.23+ for checked-in presets
- **C++ Standard:** C++20 minimum; C++23/C++26 features are optional enhancements
- **Package Managers:** vcpkg, Conan supported
- **Embedded:** Zephyr RTOS module available

## 📊 Documentation Statistics

- **Total Markdown Files:** 15 (project-specific)
- **PlantUML Diagrams:** 14
- **Example Programs:** 6
- **Documentation Words:** ~25,000
- **Code Examples:** 100+

## 🔄 Maintenance

### Regular Updates

- Update [CHANGELOG.md](CHANGELOG.md) for each release
- Regenerate diagrams after architectural changes
- Keep examples synchronized with API changes
- Update benchmarks for performance optimizations

### Documentation Reviews

- Verify all links quarterly
- Update performance metrics with benchmarks
- Add examples for frequently requested features
- Review and update TROUBLESHOOTING.md

## 📝 License

All documentation is part of MicroLA and licensed under the Apache License 2.0.
See [LICENSE](LICENSE) for details.

---

**Questions?** Check [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) or open an issue on GitHub.
