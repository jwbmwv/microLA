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

Changing runtime calibration with `set_calibration()` intentionally resets the affected estimator state so orientation, drift, and any learned magnetic-field reference are rebuilt under the new calibration frame. On a `RelativeAngleEstimator`, changing either entity calibration also clears a captured neutral reference pose because it belongs to the previous calibration frame.

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

## Compile-Checked Orientation Smoke Test

The following standalone sample verifies the public 6-axis orientation API used by an application update loop:

```cpp
#include <microla/sensor_fusion.hpp>

int main()
{
  using microla::Vec;
  using namespace microla::fusion;

  OrientationEstimator<float, DefaultImu6MahonyConfig<float>> estimator;
  const Imu6Sample<float> sample{
    0.0F,
    Vec<float, 3>(0.0F, 0.0F, 0.0F),
    Vec<float, 3>(0.0F, 0.0F, -microla::constants::gravity<float>())};

  estimator.update(sample);
  return estimator.estimate().valid ? 0 : 1;
}
```

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

## High-Dynamics Configs

The default configs are tuned for stationary-to-slow-walking activity. Wider sensor-acceptance
gates are available as drop-in replacements:

| Config | `accel_norm_min` | `accel_norm_max` | `gyro_norm_max` | `gravity_filter_alpha` | `held_orientation_confidence` |
|--------|-----------------|-----------------|----------------|----------------------|-------------------------------|
| `DefaultAccelOnlyConfig<T>` | 8.0 m/s² | 11.5 m/s² | — | 0.2 | 0.0 (invalid) |
| `HighDynamicsAccelOnlyConfig<T>` | 2.0 m/s² | 25.0 m/s² | — | 0.05 | 0.25 (degraded) |
| `DefaultImu6MahonyConfig<T>` | 8.0 m/s² | 11.5 m/s² | 35 rad/s | 0.2 | 0.0 |
| `HighDynamicsImu6MahonyConfig<T>` | 2.0 m/s² | 40.0 m/s² | 150 rad/s | 0.05 | 0.0 |
| `DefaultImu9MahonyConfig<T>` | 8.0 m/s² | 11.5 m/s² | 35 rad/s | 0.2 | 0.0 |
| `HighDynamicsImu9MahonyConfig<T>` | 2.0 m/s² | 40.0 m/s² | 150 rad/s | 0.05 | 0.0 |

### `held_orientation_confidence`

For accel-only estimators, when the accel reading is outside the valid norm window the
orientation is frozen at the last known tilt. By default `held_orientation_confidence = 0.0`,
making `quality = invalid` while the estimate is held — appropriate for safety-critical use.

Setting it to a small positive value (e.g. `0.25`) emits `quality = degraded` instead:

```cpp
struct RunningAccelConfig : microla::fusion::HighDynamicsAccelOnlyConfig<float>
{
    // Already 0.25 in HighDynamicsAccelOnlyConfig; override here for strict mode:
    // static constexpr float held_orientation_confidence = 0.0F;
};
```

Check `StatusFlag::freefall_detected` or `StatusFlag::high_linear_acceleration` to understand
why the estimate is degraded regardless of the confidence setting.

### `timestamp_needs_reset()`

A free function in the `microla::fusion` namespace that returns `true` when a `float`
timestamp approaches the 2²⁴ ms ≈ 4.7-hour precision cliff where 1 ms increments can no
longer be represented exactly. For `double` timestamps it always returns `false`.

```cpp
float timestamp_origin_s = 0.0F;
float timestamp_s = raw_timestamp_s - timestamp_origin_s;

if (microla::fusion::timestamp_needs_reset(timestamp_s)) {
  timestamp_origin_s = raw_timestamp_s;
  timestamp_s = 0.0F;
  estimator.rebase_timestamp(timestamp_s);
}
```

`rebase_timestamp()` preserves orientation, calibration, covariance, drift, and learned
magnetic-field state. For a `RelativeAngleEstimator`, rebase both entities to the same
timestamp origin with `rebase_left_timestamp()` and `rebase_right_timestamp()`.

## Compile-Time Policy Model

The algorithm is configured with policy types rather than runtime option bags. This keeps invalid combinations out of the binary and lets the compiler remove unused code.

Recommended starting points:

- `DefaultAccelOnlyConfig<T>`
- `DefaultImu6MahonyConfig<T>`
- `DefaultImu6EkfConfig<T>`
- `DefaultImu9MahonyConfig<T>`
- `DefaultImu9EkfConfig<T>`
- `HighDynamicsAccelOnlyConfig<T>` — running, jumping, moderate accel-only dynamics
- `HighDynamicsImu6MahonyConfig<T>` — running, tumbling, aerobatics (6-axis)
- `HighDynamicsImu9MahonyConfig<T>` — same with magnetometer heading
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
  - Rejects zero or implausibly tiny intervals for propagation
  - Drops out-of-order samples before they can modify estimator state

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
  - A rejected accelerometer innovation sets `StatusFlag::accel_rejected`; a rejected magnetometer innovation sets `StatusFlag::mag_rejected` and falls back to heading-with-drift observability

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

## Filtering Strategies for Improved Performance

When sampling sensors at fixed, rapid intervals (typically 100-1000 Hz), additional filtering can significantly improve orientation estimates by rejecting noise, vibration, and transient disturbances.

### Built-In Filtering

The fusion system already includes several filtering mechanisms:

**1. Gravity Low-Pass Filter**
- `gravity_filter_alpha = 0.2` applies exponential smoothing to accelerometer-derived gravity
- Formula: `gravity_new = alpha * accel_current + (1 - alpha) * gravity_old`
- Higher alpha (→ 1.0): faster response, more noise
- Lower alpha (→ 0.0): smoother output, slower response to real motion
- **Tune this first** for your vibration environment

**2. Complementary Filtering (Mahony Backend)**
- Inherently combines high-pass filtered gyro (fast response) with low-pass filtered accel/mag (absolute reference)
- `kp_accel` and `kp_mag` control the complementary filter balance
- Higher gains → trust accel/mag more (faster correction, more noise sensitivity)
- Lower gains → trust gyro more (smoother, slower correction)

**3. Kalman Filtering (MEKF Backend)**
- Process noise (`q_attitude`, `q_gyro_bias`) models how much uncertainty grows during propagation
- Measurement noise (`r_accel`, `r_mag`) models sensor uncertainty
- Innovation gating (`accel_nis_gate`, `mag_nis_gate`) rejects statistical outliers
- Automatically balances trust between propagation and correction based on noise model

**4. Magnetometer Norm Learning**
- `mag_reference_learning_alpha = 0.05` slowly adapts to local field strength
- Provides robustness to gradual magnetic environment changes

### Pre-Filtering Raw Sensor Data

For high-rate sampling with significant noise, consider filtering **before** feeding data to the fusion system:

**Moving Average Filter (Simple, Low Latency)**
```cpp
template<typename T, std::size_t WindowSize>
class MovingAverage {
    std::array<microla::Vec<T, 3>, WindowSize> buffer_{};
    std::size_t index_{0};
    bool full_{false};

public:
    auto update(const microla::Vec<T, 3>& sample) -> microla::Vec<T, 3> {
        buffer_[index_] = sample;
        index_ = (index_ + 1) % WindowSize;
        if (index_ == 0) full_ = true;
        
        microla::Vec<T, 3> sum{};
        const std::size_t count = full_ ? WindowSize : index_;
        for (std::size_t i = 0; i < count; ++i) {
            sum = sum + buffer_[i];
        }
        return sum / static_cast<T>(count);
    }
};

// Usage: 3-sample moving average on accelerometer
MovingAverage<float, 3> accel_filter;
auto filtered_accel = accel_filter.update(raw_accel);
```

**Typical window sizes:**
- 3-5 samples: Minimal smoothing, good for 100-200 Hz update rates
- 5-10 samples: Moderate smoothing, reduces high-frequency vibration
- 10-20 samples: Heavy smoothing, use only if latency tolerance is high

**Median Filter (Outlier Rejection)**
```cpp
template<typename T>
auto median_filter_3(const microla::Vec<T, 3>& a, 
                     const microla::Vec<T, 3>& b,
                     const microla::Vec<T, 3>& c) -> microla::Vec<T, 3> {
    // Per-axis median of last 3 samples
    return microla::Vec<T, 3>(
        std::max(std::min(a[0], b[0]), std::min(std::max(a[0], b[0]), c[0])),
        std::max(std::min(a[1], b[1]), std::min(std::max(a[1], b[1]), c[1])),
        std::max(std::min(a[2], b[2]), std::min(std::max(a[2], b[2]), c[2]))
    );
}
```
- Excellent for rejecting **single-sample spikes** (EMI, sensor glitches)
- Does not smooth Gaussian noise as well as moving average
- Use window size 3 or 5 (odd numbers only)

**Low-Pass IIR Filter (Exponential Moving Average)**
```cpp
template<typename T>
class LowPassFilter {
    microla::Vec<T, 3> state_{};
    T alpha_;  // Cutoff frequency parameter
    bool initialized_{false};

public:
    explicit LowPassFilter(T alpha) : alpha_(alpha) {}
    
    auto update(const microla::Vec<T, 3>& sample) -> microla::Vec<T, 3> {
        if (!initialized_) {
            state_ = sample;
            initialized_ = true;
        } else {
            state_ = sample * alpha_ + state_ * (T(1) - alpha_);
        }
        return state_;
    }
};

// Cutoff frequency calculation: alpha = dt / (dt + tau)
// where tau = 1 / (2 * pi * fc), fc = desired cutoff frequency in Hz
// Example: 100 Hz sampling, 10 Hz cutoff → tau = 0.0159, alpha ≈ 0.386
```

### Application-Specific Strategies

**High-Vibration Environments (Drones, Power Tools, Vehicles)**
- **Pre-filter accelerometer** with low-pass IIR (10-20 Hz cutoff for 100+ Hz sampling)
- **Reduce `gravity_filter_alpha`** to 0.05-0.1 for additional smoothing
- **Lower `kp_accel`** to 1.0-1.5 to reduce trust in noisy accel
- **Consider MEKF** with higher `r_accel` (0.1-0.5) to model increased measurement uncertainty
- **Increase `accel_norm_min/max` window** slightly to avoid excessive rejection from vibration peaks

**Magnetic Disturbances (Indoor, Near Electronics)**
- **Pre-filter magnetometer** with moving average (5-10 samples)
- **Widen `mag_relative_norm_tolerance`** to 0.5 to accept more variation
- **Lower `kp_mag`** to 0.5-1.0 to reduce heading correction strength
- **Or disable magnetometer** entirely and accept 6-axis drift mode

**High-Rate Clean Sensors (200-1000 Hz, Low Noise)**
- Minimal pre-filtering needed (median filter for spike rejection only)
- **Increase `gravity_filter_alpha`** to 0.3-0.5 for faster response
- **Increase `kp_accel` and `kp_mag`** to 3.0-5.0 for aggressive correction
- Fusion system's built-in filtering sufficient

**Low-Rate or Intermittent Sampling (10-50 Hz)**
- Pre-filtering provides minimal benefit (limited samples to average)
- Focus on **validity gating** to reject bad samples
- **Increase `max_dt_s`** to accommodate larger gaps (0.5-1.0 seconds)
- Accept that orientation quality will be lower

### Filtering Trade-Offs

| Approach | Latency Added | Noise Rejection | Outlier Rejection | CPU Cost |
|----------|---------------|-----------------|-------------------|----------|
| No pre-filtering | 0 ms | Minimal | None | Minimal |
| Median-3 | ~20 ms @ 100 Hz | Low | Excellent | Very low |
| Moving avg (5 samples) | ~25 ms @ 100 Hz | Good | Moderate | Very low |
| Moving avg (10 samples) | ~50 ms @ 100 Hz | Excellent | Moderate | Low |
| Low-pass IIR | ~1 sample | Good | Poor | Very low |
| Gravity filter tuning | 0 ms | Moderate | None | Zero |
| MEKF backend | 0 ms | Good (adaptive) | Good (gating) | High |

### Post-Filtering Orientation Output

You can also smooth the **final orientation estimate**, though this adds latency:

**Quaternion SLERP Smoothing**
```cpp
// Spherical linear interpolation between consecutive orientations
auto smooth_quat = microla::slerp(prev_orientation, current_orientation, alpha);
// alpha = 0.3-0.5 typical for moderate smoothing
```

**⚠️ Warning:** Post-filtering orientation adds latency and can create lag during fast rotations. Only use when you need extremely smooth output for visualization or control systems with low bandwidth requirements.

### Recommended Starting Point

For typical embedded IMU applications (100-200 Hz sampling):

1. **Start with defaults** - built-in filtering often sufficient
2. **Add median-3 filter** on accelerometer if you see spike artifacts
3. **Tune `gravity_filter_alpha`** down (0.05-0.1) if vibration is severe
4. **Adjust `kp_accel` and `kp_mag`** based on noise vs. responsiveness needs
5. **Switch to MEKF** only if you need covariance-aware correction
6. **Add moving average** (5-10 samples) only if previous steps insufficient

Monitor `StatusFlag` rejection rates - if you're rejecting >30% of samples due to norm violations, you likely need better pre-filtering rather than wider validity gates.

## Known Limitations and Failure Modes

### Fundamental Assumptions

IMU-based orientation estimation relies on a **critical assumption**: the accelerometer measures gravity. In reality, accelerometers measure **net force** (gravity + linear acceleration + centripetal acceleration). The system assumes **quasi-static motion** where linear acceleration ≈ 0.

This assumption breaks down in several common scenarios:

### Freefall and Zero-G Environments

**Problem:** During freefall, the accelerometer reads near zero.

- Freefall: gravity - drag ≈ 0 m/s² (well below `accel_norm_min = 8.0`)
- **All accelerometer samples are rejected** (`StatusFlag::accel_rejected` + `StatusFlag::freefall_detected`)
- System enters **gyro-only propagation** mode
- **Tilt reference is completely lost** - only gyro integration remains
- Rapid drift accumulation: at default `drift_rate_without_heading_rad_per_s = 0.02`, a 60-second freefall accumulates **1.2 radians (69°) of drift**
- `Observability` drops from `full_3d` to effectively `none`
- `SolutionQuality` degrades to `invalid` or `degraded`

**Applications affected:** Skydiving, parabolic flight, space applications, dropped objects

**Recovery:** When acceleration returns to ~1g (parachute deployment, landing), tilt reference recovers. Recovery speed depends on `kp_accel` and `gravity_filter_alpha`.

### High-G Impacts and Extreme Acceleration

**Problem:** The accelerometer reads gravity + linear acceleration.

- Impact landing: 3-5g → **30-50 m/s²** (above `accel_norm_max = 11.5`)
- Aggressive vehicle maneuvering: 2-3g sustained
- **Samples rejected during high-acceleration events** (`StatusFlag::accel_rejected` + `StatusFlag::high_linear_acceleration`)
- Brief propagation-only mode until acceleration settles

**Sustained high acceleration:**
- If continuous motion keeps you outside [8.0, 11.5] m/s² range
- Extended rejection period → drift accumulation
- `quality` oscillates between `nominal` and `degraded`

**Applications affected:** Helmets during running/tumbling, racing vehicles, roller coasters, impact monitoring

**Tuning tradeoff:** Widening the gate (e.g., `accel_norm_max = 30.0`) accepts more contaminated samples, reducing rejection frequency but allowing corrupted gravity estimates that produce incorrect tilt.

### Running and Rhythmic Motion

**Moderate but periodic violation:**

- Running produces 2-3g peaks during foot strike
- **Marginally within or outside** the [8.0, 11.5] m/s² window
- Intermittent acceptance/rejection creates **oscillating quality** tied to gait cycle
- Some tilt corrections accepted, others rejected
- Result: usable but noisy orientation estimates

### High Angular Rates

**Problem:** Extremely fast rotation exceeds sensor and validation limits.

- Aggressive tumbling can exceed `gyro_norm_max = 35 rad/s` (2000 deg/s)
- **Gyro samples rejected** (`StatusFlag::gyro_rejected` + `StatusFlag::high_rotation_rate`)
- **No propagation occurs** - orientation estimate freezes
- Even if software gate allows it, physical sensor may saturate first

**Typical IMU ranges:** ±250 to ±2000 deg/s depending on configuration. Choose hardware range to match expected angular rates and adjust `gyro_norm_max` accordingly.

**Applications affected:** Gymnastics, aggressive drone maneuvers, spinning tools

### Magnetometer Disturbances

**Problem:** Magnetometers measure the local magnetic field, which is easily distorted.

**Hard-iron distortion (static):**
- Nearby ferromagnetic materials (buckles, screws, batteries)
- Electronics (processors, motors, wiring)
- Can be calibrated out via `SensorCalibration::mag_bias` and `mag_soft_iron`

**Soft-iron distortion (field-dependent):**
- Ferromagnetic materials create field-dependent distortion
- Static calibration (3×3 `mag_soft_iron` matrix) handles **constant** mounting geometry
- **Dynamic distortion** (moving electronics, orientation-dependent metal) cannot be fully calibrated

**Rejection scenarios:**
- Field strength outside learned norm ± `mag_relative_norm_tolerance` (default ±35%)
- Directional error exceeds `mag_alignment_max_error_rad` (2.356 rad / 135°)
- Results in `StatusFlag::mag_rejected`
- System falls back to `heading_with_drift` mode (behaves like 6-axis IMU)
- Heading drifts at ~`0.02 rad/s = 1.15°/s`

**Applications affected:** Helmet-mounted IMUs (electronics, NVGs, cameras), indoor environments, near steel structures, vehicles with motors

**Mitigation:** For magnetically-polluted environments, disable magnetometer correction entirely (`enable_mag_correction = false`) and accept 6-axis behavior rather than risk bad heading corrections.

### Summary Table

| Scenario | Accelerometer | Gyroscope | Magnetometer | Tilt Observable? | Heading Observable? | Typical Quality | New Sub-flags |
|----------|---------------|-----------|--------------|------------------|---------------------|-----------------|---------------|
| Freefall | Rejected (0g) | OK | OK | **No** | No (gyro-only) | `invalid` | `freefall_detected` |
| High-G Impact | Rejected (>3g) | OK | OK | **No** (temporarily) | Depends on mag | `invalid` → recovers | `high_linear_acceleration` |
| Tumbling | Rejected (high-g) | Rejected (high rate) | OK | **No** | **No** | `invalid` | `freefall_detected` / `high_linear_acceleration`, `high_rotation_rate` |
| Running | Intermittent | OK | OK | Intermittent | Yes (if mag clean) | `usable` ↔ `nominal` | `high_linear_acceleration` at footstrike |
| Near Electronics | OK | OK | Rejected (distorted) | Yes | **No** (drift mode) | `usable` | — |
| Quasi-Static | OK | OK | OK | Yes | Yes | `nominal` | — |

### Design Response

Rather than pretending these scenarios produce valid output, microLA's sensor fusion explicitly reports degradation through:

- **Observability levels** (`none`, `tilt_only`, `heading_with_drift`, `full_3d`)
- **Quality grades** (`invalid`, `degraded`, `usable`, `nominal`)
- **Status flags** (sensor rejection, drift limits, propagation-only mode):
  - `accel_rejected` + `freefall_detected` — accelerometer below minimum (free-fall / zero-g)
  - `accel_rejected` + `high_linear_acceleration` — accelerometer above maximum (impact / high-g)
  - `gyro_rejected` + `high_rotation_rate` — gyroscope saturated (rapid tumbling / aerobatics)
  - `mag_rejected` — magnetometer distorted or outside learned norm
  - `propagation_only` — gyro-only integration, no measurement correction
- **Drift estimates** (`estimated_drift_rad`)
- **Confidence scores** (0-1 scale)

This allows safety-critical applications to detect unreliable estimates and respond appropriately (fallback modes, increased margins, operator warnings).

## Current Scope Notes

- Accelerometer-only entities support tilt observability, not absolute heading observability.
- Hinge and swing outputs are currently treated conservatively: they require at least heading-with-drift observability for a valid scalar result.
- The API is designed so additional observability logic and more specialized biomechanical constraints can be added later without changing the public result format.
