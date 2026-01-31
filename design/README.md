# MicroLA Design Documentation

This directory contains PlantUML diagrams documenting the architecture and design of MicroLA.

## Diagrams

### 1. Class Hierarchy (`class_hierarchy.puml`)
Shows the main classes (Vec, Mat, SquareMat, Quaternion) and their relationships.

### 2. SIMD Optimization Strategy (`simd_optimization.puml`)
Documents the three-tier SIMD optimization approach (NEON → CMSIS → Generic).

### 3. C++ Standard Features (`cpp_standard_features.puml`)
Illustrates progressive enhancement from C++11 through C++26.

### 4. Memory Layout (`memory_layout.puml`)
Details memory organization, alignment, and SIMD benefits.

### 5. Module Structure (`module_structure.puml`)
Shows header dependencies and build system integration.

## Generating Diagrams

```bash
# Using PlantUML 1.2025.10 with Batik 1.19
java -jar C:\Users\think\bin\plantuml.jar \
  -DPLANTUML_BATIK=C:\Users\think\bin\batik-1.19 \
  design/*.puml

# Generate SVG
java -jar C:\Users\think\bin\plantuml.jar -tsvg design/*.puml
```
