# Sensor Fusion and Relative Angle Estimation

## Overview

MicroLA now provides a relative sensor-fusion API in `microla::fusion` for estimating the angle between two entities when each side may provide one of these sensor sets:

- Accelerometer only
- 6-axis IMU (`gyro + accel`)
- 9-axis IMU (`gyro + accel + mag`)

The API is designed for embedded use:

- No dynamic allocation
- Compile-time policy types for gating and algorithm selection
- Runtime calibration structs for board-specific alignment and bias correction
- Explicit drift and quality reporting instead of pretending every output is fully observable

## Main Types

- `OrientationEstimator<T, Config>`: per-entity orientation state estimator
- `RelativeAngleEstimator<T, LeftConfig, RightConfig, RelativeConfig>`: combines two entity estimates into a relative-angle result
- `AccelSample<T>`, `Imu6Sample<T>`, `Imu9Sample<T>`: strongly typed sample inputs
- `SensorCalibration<T>`: runtime mounting and bias calibration
- `RelativeAngleResult<T>`: relative quaternion, selected scalar output, drift estimate, confidence, and status flags

Changing runtime calibration with `set_calibration()` intentionally resets the affected estimator state so orientation, drift, and any learned magnetic-field reference are rebuilt under the new calibration frame.

## Units Policy

The sensor-fusion API uses explicit scalar SI conventions instead of a mandatory units-library dependency.

- Timestamps are in seconds
- Gyroscope samples are in rad/s
- Accelerometer samples are in m/s^2
- Relative angles and drift estimates are in radians
- Magnetometer samples may use any consistent field-strength unit after calibration (for example uT)
- The estimator learns the calibrated magnetic-field norm online, so those magnetometer units do not need to match the default policy constants

## Maintained Example

The shipped example in `examples/sensor_fusion.cpp` uses the public API directly rather than a private complementary filter. It demonstrates:

- A mixed sensor pair: left entity on 9-axis IMU, right entity on 6-axis IMU
- Compile-time policy types for per-entity fusion and relative-angle extraction
- Neutral reference capture through `capture_reference_pose()`
- Default hinge-twist output plus runtime override to `tilt_angle` and `heading_delta`
- Drift-aware diagnostics when one side lacks absolute heading correction

That example is also executed as a CTest smoke test when examples and tests are enabled together, so the documented flow is exercised in CI instead of being compile-only.

## Backend Choices

Each entity config chooses one backend at compile time through `Config::backend`.

- `FusionBackend::mahony`
  - Lowest-footprint default
  - Good for most embedded targets
  - Uses proportional correction from gravity and magnetic reference vectors
  - Supports gyro-bias adaptation when the entity is stationary

- `FusionBackend::mekf`
  - Optional multiplicative EKF-style backend
  - Tracks covariance and applies innovation gating
  - Better fit when you want covariance-aware correction and more structured tuning
  - Costs more CPU and matrix math than Mahony

`SensorModel::accel_only` intentionally does not support the MEKF backend because there is no useful gyro-driven propagation state to estimate.

## Scalar Outputs

The relative estimator always computes a relative quaternion and several derived angles. The user may select the primary scalar output at runtime through `compute_scalar(mode)` when the relative config allows it.

Available primary scalar modes:

- `PrimaryScalarOutput::shortest_3d_angle`
  - Magnitude of the smallest 3D rotation from left entity to right entity
  - Best for generic rigid-body comparison

- `PrimaryScalarOutput::tilt_angle`
  - Angle between the two entities' down axes
  - Best for accelerometer-only or tilt-dominant use cases

- `PrimaryScalarOutput::heading_delta`
  - Relative heading using the configured body heading axes projected onto the horizontal plane
  - Requires heading observability or drift-enabled policy

- `PrimaryScalarOutput::hinge_twist`
  - Signed twist angle extracted about `RelativeConfig::hinge_axis_left()`
  - Best for hinge-like joints

- `PrimaryScalarOutput::swing_angle`
  - Magnitude of the swing component after swing-twist decomposition

- `PrimaryScalarOutput::roll_delta`
- `PrimaryScalarOutput::pitch_delta`
- `PrimaryScalarOutput::yaw_delta`
  - Euler-angle deltas extracted from the relative quaternion
  - Useful for debugging or convention-specific outputs

## Observability Model

The API distinguishes between what is mathematically observable and what is merely being propagated.

- `Observability::none`
  - The requested scalar is not supportable for the current sensor combination and estimator state

- `Observability::tilt_only`
  - Gravity alignment is observable, but heading is not
  - Typical for accelerometer-only pairs

- `Observability::heading_with_drift`
  - Heading-dependent outputs are available, but they drift because one or both entities do not currently have a valid absolute heading correction
  - Typical for 6-axis IMU pairs or 9-axis IMUs with rejected magnetometer updates

- `Observability::full_3d`
  - Full orientation is currently constrained by valid tilt and heading information

If the chosen scalar requires heading but only drift is available, the estimator can still return a value when `RelativeConfig::allow_drift_with_quality_flags` is `true`. In that case the result carries:

- `estimated_drift_rad`
- `confidence`
- `quality`
- `StatusFlag::drift_exceeds_nominal` when applicable

## Compile-Time Policy Model

The algorithm is configured with policy types rather than runtime option bags. This keeps invalid combinations out of the binary and lets the compiler remove unused code.

Recommended starting points:

- `DefaultAccelOnlyConfig<T>`
- `DefaultImu6MahonyConfig<T>`
- `DefaultImu6EkfConfig<T>`
- `DefaultImu9MahonyConfig<T>`
- `DefaultImu9EkfConfig<T>`
- `DefaultRelativeAngleConfig<T>`

Create your own policy by deriving from one of these and overriding only the members you need.

Example:

```cpp
struct ThighConfig : microla::fusion::DefaultImu9MahonyConfig<float>
{
    static constexpr float kp_accel = 2.0F;
    static constexpr float kp_mag = 1.0F;
    static constexpr float accel_norm_min = 8.5F;
    static constexpr float accel_norm_max = 11.0F;
};

struct ShankConfig : microla::fusion::DefaultImu6MahonyConfig<float>
{
    static constexpr float drift_rate_without_heading_rad_per_s = 0.03F;
};

struct KneeConfig : microla::fusion::DefaultRelativeAngleConfig<float>
{
    static constexpr microla::fusion::PrimaryScalarOutput default_output =
        microla::fusion::PrimaryScalarOutput::hinge_twist;

    static auto hinge_axis_left() noexcept -> microla::Vec<float, 3>
    {
        return microla::Vec<float, 3>(0.0F, 0.0F, 1.0F);
    }
};
```

## Tuning Knobs

The following knobs are implemented in the public policy types and affect runtime behavior.

### Timing and freshness

- `min_dt_s`
  - Rejects zero or implausibly tiny intervals
  - Protects against duplicate timestamps and bad packet ordering

- `max_dt_s`
  - Suppresses propagation across large gaps
  - Prevents the estimator from blindly integrating stale rates for too long

- `max_sample_age_s`
  - Marks an entity stale if its latest estimate is too old relative to the latest pair timestamp

- `RelativeConfig::max_pair_skew_s`
  - Degrades the relative result when left and right estimates are too far apart in time

- `RelativeConfig::max_alignment_horizon_s`
  - Allows `compute()` to predict a recent entity state forward to the latest pair timestamp before extracting the relative angle
  - This compensates recent skew when a valid angular-rate state is available, while still preserving the skew flags and confidence penalties

### Validity gates

- `accel_norm_min` and `accel_norm_max`
  - Define when accelerometer magnitude is considered trustworthy as a gravity measurement
  - Narrower ranges reject more motion contamination but may reject aggressive maneuvers

- `gyro_norm_max`
  - Rejects saturated or physically implausible angular-rate samples

- `expected_mag_norm`, `mag_norm_min`, and `mag_norm_max`
  - Optional bootstrap hints when your calibrated magnetometer values already use a known field-strength unit
  - The default estimator path also learns the local field norm online, so these no longer need to match a fixed uT-scale deployment

- `mag_relative_norm_tolerance`
  - Allowed fractional deviation from the learned magnetic-field norm before a sample is rejected

- `mag_reference_learning_alpha`
  - Update rate for the learned magnetic-field norm

- `mag_alignment_max_error_rad`
  - Directional gate applied while heading is currently trusted
  - Rejects large magnetic direction jumps without blocking later heading reacquisition from a previously drift-only state

- `stationary_gyro_norm_max`
- `stationary_accel_error_max`
  - Define when the entity is considered stationary enough to adapt gyro bias safely

### Mahony backend gains

- `kp_accel`
  - Strength of gravity-based correction
  - Higher values recover tilt faster but react more strongly to contaminated acceleration

- `kp_mag`
  - Strength of magnetic heading correction
  - Higher values recover heading faster but become more sensitive to magnetic disturbance

- `ki_gyro_bias`
  - Bias adaptation speed
  - Higher values learn bias faster but can absorb real motion if stationary detection is too permissive

- `gravity_filter_alpha`
  - Low-pass coefficient for accelerometer-derived gravity estimate
  - Lower values reject vibration better but slow the tilt response

- `drift_rate_without_heading_rad_per_s`
  - Heuristic drift growth used when heading is being propagated without absolute correction

- `drift_confidence_limit_rad`
  - Drift level where confidence is heavily penalized

### MEKF backend noise model

- `q_attitude`
  - Attitude process noise
  - Larger values trust gyro propagation less and allow faster correction

- `q_gyro_bias`
  - Gyro-bias process noise
  - Larger values let the estimated bias move faster

- `r_accel`
  - Accelerometer measurement variance
  - Larger values reduce the influence of gravity corrections

- `r_mag`
  - Magnetometer measurement variance
  - Larger values reduce the influence of heading corrections

- `accel_nis_gate`
- `mag_nis_gate`
  - Innovation gates used to reject improbable vector measurements

### Frame conventions

- `body_down_axis()`
  - Defines which body-frame axis corresponds to local down when the entity is level

- `body_heading_axis()`
  - Defines which body-frame axis is projected into the horizontal plane for heading comparison

- `world_gravity_direction()`
  - Defines the world-frame gravity direction used by the estimator

- `world_magnetic_reference()`
  - Defines the world-frame magnetic reference direction used for heading alignment

### Relative-output policy

- `default_output`
  - The primary scalar returned by `compute()`

- `allow_runtime_output_override`
  - Enables `compute_scalar(mode)` for caller-selected outputs

- `allow_drift_with_quality_flags`
  - Allows heading-dependent outputs to be returned when only drifting heading is available

- `apply_reference_pose`
  - Applies a captured neutral pose before scalar extraction

- `hinge_axis_left()` and `hinge_axis_right()`
  - Define the hinge or twist axis for swing-twist outputs
  - Hinge-oriented scalar outputs expect those two axes to describe the same physical joint axis after frame transformation; inconsistent axes are rejected

- `min_confidence_to_publish`
  - Confidence floor below which the relative result is marked invalid

- `nominal_drift_limit_rad`
  - Drift level above which the result is explicitly flagged as degraded

## Runtime Calibration

Use `SensorCalibration<T>` for per-board calibration and mounting alignment:

- `sensor_to_body`
- `accel_bias`, `accel_scale`
- `gyro_bias`, `gyro_scale`
- `mag_bias`, `mag_soft_iron`

These are intentionally runtime-configurable because they differ across manufactured units and mounting arrangements.

Runtime calibration is now validated before use:

- `sensor_to_body` must be finite and non-zero; finite non-unit quaternions are normalized automatically
- `accel_scale` and `gyro_scale` must be finite and non-zero per axis
- `mag_soft_iron` must be finite and invertible
- Invalid calibration is surfaced through `StatusFlag::calibration_invalid`

The API also validates policy and frame consistency at runtime. Invalid or inconsistent world-frame conventions, degenerate axes, or malformed policy thresholds are surfaced through `StatusFlag::configuration_invalid`.

## Typical Usage

```cpp
using KneeEstimator = microla::fusion::RelativeAngleEstimator<
    float,
    ThighConfig,
    ShankConfig,
    KneeConfig>;

KneeEstimator estimator;

estimator.update_left(left_sample);
estimator.update_right(right_sample);

auto primary = estimator.compute();
auto tilt = estimator.compute_scalar(microla::fusion::PrimaryScalarOutput::tilt_angle);
auto heading = estimator.compute_scalar(microla::fusion::PrimaryScalarOutput::heading_delta);
```

For a complete runnable program built around that pattern, see `examples/sensor_fusion.cpp`.

If the requested output is only available with drift, the result remains usable when allowed by policy, but it must be interpreted together with:

- `result.primary.observability`
- `result.primary.estimated_drift_rad`
- `result.primary.confidence`
- `result.primary.quality`
- `result.primary.flags`

## Current Scope Notes

- Accelerometer-only entities support tilt observability, not absolute heading observability.
- Hinge and swing outputs are currently treated conservatively: they require at least heading-with-drift observability for a valid scalar result.
- The API is designed so additional observability logic and more specialized biomechanical constraints can be added later without changing the public result format.
