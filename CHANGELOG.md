# Changelog

All notable changes are summarized here.

## [0.0.4] - 2026-10-10

- **Ray-triangle scaling**: Include the ray direction norm in both triangle intersection parallelism checks so rescaling the direction preserves hits and near-parallel rejection.
- **Fusion policies and relative angles**: Honor disabled bias estimation and disabled acceleration correction during recovery; keep observable tilt valid independently of heading drift; time-align both entities when capturing a neutral reference.
- **Numerical edge cases**: Make KF/EKF prediction transactional; correct IEEE ULP distances and non-finite comparisons; scale public vector/quaternion normalization, rank, QR, and checked-inverse diagnostics; reject invalid Cholesky input; use fraction-free integral determinants; make quaternion distance sign/scale invariant; correct sphere-ray intervals and scale-relative triangle intersections; delegate large-angle fast trigonometry to standard math.
- **Multiplication performance**: Replace strided float gather/reduction products with contiguous row accumulation, including the NEON 4x4 kernel. Add paired library/reference benchmarks at dimensions 4, 6, 32, and 64.
- **Documentation accuracy**: Replace the inconsistent additive attitude-EKF recipe with the manifold MEKF API; correct nonexistent methods, embedded overload guidance, alignment diagrams, and unsupported optimization claims.
- **Sensor-fusion recovery**: Corrected Mahony gravity/magnetic feedback direction, rejected stale samples before state changes, retained unobservable heading during gravity-only dropout correction, and stopped reporting valid gyro input as rejected when timing suppresses propagation.
- **Sensor-fusion freshness**: Held IMU outputs now report invalid quality and zero confidence with `orientation_held`, retain their actual state timestamp, and disable prediction. Processed-sample timing prevents recovery from integrating rejected gaps; rebasing preserves held-state age. Explicit accel-only held-confidence policies remain supported.
- **MEKF consistency**: Measurement-only recovery now honors normal covariance gains and NIS gates. Apply the right-error covariance reset after Joseph updates, validate corrections before committing, and learn magnetic norms only from accepted corrections. Added a read-only `error_covariance()` accessor for MEKF IMU estimators.
- **Rotations**: Corrected matrix-to-quaternion conversion and antipodal SLERP; adapted CMSIS-DSP quaternion products to its scalar-first single-product API.
- **Linear algebra**: Corrected late LU pivots, complete QR convergence checks, scale-aware and alias-safe inversion, and double Frobenius norms. Replaced normal-equation SVD with scaled one-sided Jacobi, full orthogonal bases, and an optional convergence output.
- **Filtering**: Reject non-finite measurements and results without mutating state, report failed NIS evaluation as infinity, and use checked inverses in both host and embedded updates. Added allocation-free EKF residual callbacks for wrapped measurements.
- **Math and geometry**: Corrected logarithm/power accuracy, bounded exponential and angle reduction, integer vector division, signed safe-division overflow, scaled norm/condition diagnostics, frustum extraction, and triangle bounds.
- **Contracts**: Restored standalone public-header compilation and float constexpr arithmetic/factories; compile each public header independently in both test targets.
- **Sensor fusion**: Preserve magnetic direction across microtesla, gauss, tesla, and extreme input scales with scale-safe normalization. Reject fields without a usable horizontal component and collinear configured magnetic/gravity references. Absolute magnetic limits require `enforce_mag_norm_limits = true` with limits expressed in calibrated input units.
- **Documentation and tests**: Corrected constant-turn dynamics/Jacobians and radar residual/origin handling; added focused regressions and enforced meaningful approximation tolerances.

## [0.0.3] - 2026-07-25

- **Sensor fusion**: Corrected timestamp, calibration, reference-pose, and measurement-rejection behavior with deterministic trace-replay coverage.
- **Embedded validation**: Added target compile/resource reports, alignment sanitizer coverage, and a Cortex-M4F Renode runtime CI lane.
- **Release engineering**: Added a release contract, explicit validation boundaries, and an ABI-specific estimator-state planning tool.

## [0.0.2] - 2026-05-31

Comprehensive code review and quality improvements:

- **Safety**: NaN propagation for error detection (safe_sqrt, quaternion inverse, division by zero)
- **Performance**: Cached identity matrices, analytical 4×4 inverse (9.22× faster), cache-optimized matrix multiply, AVX-512 support, branch prediction hints with [[likely]]/[[unlikely]]
- **Correctness**: Kalman filter covariance symmetry, Ray-AABB epsilon checks, PlantUML diagram fixes
- **Robustness**: Overflow protection in hypot3, magic constant extraction, timestamp precision warnings
- **Documentation**: Enhanced API docs, sensor fusion failure modes guide, filtering strategies
- **CI/CD**: Automated code snippet verification, Doxygen generation pipeline
- **Testing**: 537 tests passing, updated to reflect NaN propagation behavior

## [0.0.1] - 2026-01-31

Initial release: header-only C++ linear algebra library for embedded and real-time systems.

- Header-only templates: `Vec<T,N>`, `Mat<T,R,C>`, `Quaternion<T>`
- Zero dynamic allocation and POD-compatible types
- SIMD-aware implementations with platform-specific fast paths
- Compile-time rotation helpers and constexpr-friendly utilities
- Safety utilities (safe math, saturating arithmetic)
- Basic benchmarks and tests included

Current Version: **0.0.4**
