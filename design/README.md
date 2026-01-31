# MicroLA Design Documentation

This directory contains PlantUML diagrams documenting the architecture and design of MicroLA.

## Diagrams

### 1. Class Hierarchy (`class_hierarchy.puml`)

Shows the main types (Vec, Mat, SquareMat alias, Quaternion) and their relationships.

### 2. SIMD Optimization Strategy (`simd_optimization.puml`)

Documents the current SIMD strategy across NEON, CMSIS-DSP, AVX, RISC-V, and scalar fallback paths.

### 3. C++ Standard Features (`cpp_standard_features.puml`)

Illustrates progressive enhancement from C++17 through C++26.

### 4. Memory Layout (`memory_layout.puml`)

Details memory organization, alignment, and SIMD benefits.

### 5. Module Structure (`module_structure.puml`)

Shows header dependencies and build system integration.

## Generating Diagrams

```bash
# Generate PNG/SVG/PDF for both design and example diagrams from the repository root
bash scripts/generate_diagrams.sh
```
