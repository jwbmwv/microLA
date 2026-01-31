This folder contains PlantUML sequence diagrams for each example source file.

- `basic_usage.puml`: Basic vector & matrix creation and operations.
- `advanced_features_demo.puml`: Expression templates, constexpr paths, and SIMD evaluation.
- `constexpr_rotations.puml`: Compile-time quaternion and rotation examples.
- `graphics_pipeline.puml`: Typical model-view-projection flow for rendering.
- `kalman_demo.puml`: Predict/update Kalman filter loop.
- `robotics_kinematics.puml`: Forward and inverse kinematics flow.
- `sensor_fusion.puml`: Relative 9-axis plus 6-axis fusion flow with drift-aware angle output.
- `test_defaults.puml`: Tests for default settings and tolerances.
- `test_new_features.puml`: Smoke tests for new/experimental APIs.

How to render diagrams

1. Install PlantUML and Graphviz.
2. From this directory run, for example:

```bash
plantuml -tpng basic_usage.puml
plantuml -tsvg basic_usage.puml
plantuml -tpdf basic_usage.puml
```

Or use the helper scripts in `scripts/` to render all diagrams at once from the repo root:

```bash
bash scripts/generate_diagrams.sh
```

On Windows (cmd):

```
scripts\\generate_diagrams.bat
```

Notes

- The diagrams are high-level; consult the corresponding `.cpp` example for implementation details.
- The helper scripts regenerate PNG, SVG, and PDF outputs for both `examples/` and `design/`.
