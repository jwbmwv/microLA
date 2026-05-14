// SPDX-License-Identifier: Apache-2.0
/// @file sensor_fusion.hpp
/// @brief Relative sensor-fusion utilities for angle estimation between two entities
/// @details Provides embedded-friendly orientation estimators for accelerometer-only, 6-axis IMU,
///          and 9-axis IMU entities, plus a relative-angle estimator that can report a selectable
///          scalar output together with drift and quality metadata.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#pragma once

#include "compiler_features.hpp"
#include "constants.hpp"
#include "matrix.hpp"
#include "quaternion.hpp"
#include "vector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace microla
{
namespace fusion
{

/// @brief Supported per-entity sensor capability models.
enum class SensorModel
{
    accel_only,
    imu6,
    imu9
};

/// @brief Supported per-entity fusion backends.
enum class FusionBackend
{
    mahony,
    mekf
};

/// @brief Selects which scalar angle should be treated as the primary output.
enum class PrimaryScalarOutput
{
    shortest_3d_angle,
    tilt_angle,
    heading_delta,
    hinge_twist,
    swing_angle,
    roll_delta,
    pitch_delta,
    yaw_delta
};

/// @brief Describes what part of the relative orientation is currently observable.
enum class Observability
{
    none,
    tilt_only,
    heading_with_drift,
    full_3d
};

/// @brief Coarse quality grade for the current estimate.
enum class SolutionQuality
{
    invalid,
    degraded,
    usable,
    nominal
};

/// @brief Status flags reported by entity and relative estimators.
enum class StatusFlag : std::uint32_t
{
    none = 0U,
    sample_time_invalid = 1U << 0,
    gyro_rejected = 1U << 1,
    accel_rejected = 1U << 2,
    mag_rejected = 1U << 3,
    propagation_only = 1U << 4,
    startup_not_initialized = 1U << 5,
    left_sample_stale = 1U << 6,
    right_sample_stale = 1U << 7,
    pair_time_skew_exceeded = 1U << 8,
    heading_unobservable = 1U << 9,
    drift_exceeds_nominal = 1U << 10,
    output_not_supported = 1U << 11,
    calibration_invalid = 1U << 12,
    configuration_invalid = 1U << 13
};

using StatusFlags = std::uint32_t;

/// @brief Convert a single flag to its underlying mask value.
constexpr auto flag_mask(StatusFlag flag) noexcept -> StatusFlags
{
    return static_cast<StatusFlags>(flag);
}

/// @brief Combine two status flags.
constexpr auto operator|(StatusFlag lhs, StatusFlag rhs) noexcept -> StatusFlags
{
    return flag_mask(lhs) | flag_mask(rhs);
}

/// @brief Combine an existing mask with an additional flag.
constexpr auto operator|(StatusFlags lhs, StatusFlag rhs) noexcept -> StatusFlags
{
    return lhs | flag_mask(rhs);
}

/// @brief Checks whether a specific flag is present in a mask.
constexpr auto has_flag(StatusFlags flags, StatusFlag flag) noexcept -> bool
{
    return (flags & flag_mask(flag)) != 0U;
}

/// @brief Adds a flag to a mutable mask.
inline void set_flag(StatusFlags& flags, StatusFlag flag) noexcept
{
    flags |= flag_mask(flag);
}

/// @brief Accelerometer-only sample.
/// @details `timestamp_s` is in seconds and `accel` is expected in m/s^2 in the sensor frame.
template<typename T>
struct AccelSample
{
    T timestamp_s{};
    Vec<T, 3> accel{};
};

/// @brief 6-axis IMU sample (gyro + accel).
/// @details `timestamp_s` is in seconds, `gyro` is in rad/s, and `accel` is in m/s^2.
template<typename T>
struct Imu6Sample
{
    T timestamp_s{};
    Vec<T, 3> gyro{};
    Vec<T, 3> accel{};
};

/// @brief 9-axis IMU sample (gyro + accel + mag).
/// @details `timestamp_s` is in seconds, `gyro` is in rad/s, `accel` is in m/s^2, and `mag`
///          uses any consistent magnetic-field unit after calibration (for example uT).
template<typename T>
struct Imu9Sample
{
    T timestamp_s{};
    Vec<T, 3> gyro{};
    Vec<T, 3> accel{};
    Vec<T, 3> mag{};
};

/// @brief Runtime calibration for one entity's sensor package.
/// @details These values are device-specific and therefore intentionally runtime-configurable,
///          while the gating and algorithm-policy knobs remain compile-time constants.
template<typename T>
struct SensorCalibration
{
    Quaternion<T> sensor_to_body;
    Vec<T, 3> accel_bias;
    Vec<T, 3> accel_scale;
    Vec<T, 3> gyro_bias;
    Vec<T, 3> gyro_scale;
    Vec<T, 3> mag_bias;
    Mat<T, 3, 3> mag_soft_iron;

    SensorCalibration() noexcept
        : sensor_to_body(Quaternion<T>::identity()), accel_bias(), accel_scale(T(1), T(1), T(1)), gyro_bias(),
          gyro_scale(T(1), T(1), T(1)), mag_bias(), mag_soft_iron(Mat<T, 3, 3>::identity())
    {
    }
};

/// @brief Per-entity orientation estimate with quality and drift metadata.
template<typename T>
struct OrientationEstimate
{
    Quaternion<T> orientation = Quaternion<T>::identity();
    T confidence{};
    T estimated_drift_rad{};
    T timestamp_s{};
    Observability observability = Observability::none;
    SolutionQuality quality = SolutionQuality::invalid;
    StatusFlags flags{};
    bool valid{};
    bool heading_referenced{};
};

/// @brief Result for one scalar angle extraction mode.
/// @details All angle and drift fields are reported in radians.
template<typename T>
struct ScalarAngleResult
{
    PrimaryScalarOutput mode = PrimaryScalarOutput::shortest_3d_angle;
    T angle_rad{};
    T confidence{};
    T estimated_drift_rad{};
    Observability observability = Observability::none;
    SolutionQuality quality = SolutionQuality::invalid;
    StatusFlags flags{};
    bool valid{};
};

/// @brief Full relative-angle result including diagnostics and secondary outputs.
/// @details All angle outputs are in radians and `sample_skew_s` is in seconds.
template<typename T>
struct RelativeAngleResult
{
    OrientationEstimate<T> left_entity{};
    OrientationEstimate<T> right_entity{};
    Quaternion<T> q_left_to_right = Quaternion<T>::identity();
    ScalarAngleResult<T> primary{};
    Vec<T, 3> relative_euler_rad{};
    T shortest_3d_angle_rad{};
    T tilt_angle_rad{};
    T heading_delta_rad{};
    T hinge_twist_rad{};
    T swing_angle_rad{};
    T sample_skew_s{};
    bool reference_active{};
    bool valid{};
};

/// @brief Shared compile-time policy knobs for one entity orientation estimator.
/// @details Derive your own policy from one of the default configs below and override only the
///          members you need to tune. Every value in this base is intentionally static so the
///          compiler can remove unused code paths and enforce configuration restrictions.
template<typename T>
struct OrientationConfigBase
{
    static_assert(std::is_floating_point<T>::value,
                  "Sensor-fusion orientation configs require a floating-point scalar type.");

    /// @brief Declares the sensor capability of the entity.
    static constexpr SensorModel sensor_model = SensorModel::imu6;

    /// @brief Selects the fusion backend.
    /// @details Use Mahony for the lowest-footprint default backend. Use MEKF when you want a
    ///          covariance-tracking backend with innovation gating and are willing to spend more
    ///          CPU cycles and matrix math.
    static constexpr FusionBackend backend = FusionBackend::mahony;

    /// @brief Enables gyro bias adaptation when the entity is detected as stationary.
    static constexpr bool estimate_gyro_bias = true;

    /// @brief Enables accelerometer-based gravity correction.
    static constexpr bool enable_accel_correction = true;

    /// @brief Enables magnetometer-based heading correction.
    static constexpr bool enable_mag_correction = true;

    /// @brief Lower bound on acceptable sample interval.
    /// @details Non-positive or excessively tiny intervals are usually timestamp duplication or
    ///          ordering bugs; such samples are ignored for propagation.
    static constexpr T min_dt_s = static_cast<T>(1e-6);

    /// @brief Upper bound on acceptable sample interval.
    /// @details Large gaps imply stale state; the update remains measurable, but propagation is
    ///          suppressed and quality is degraded.
    static constexpr T max_dt_s = static_cast<T>(0.2);

    /// @brief Maximum age allowed before the entity estimate is treated as stale.
    static constexpr T max_sample_age_s = static_cast<T>(0.05);

    /// @brief Expected accelerometer norm when the sensor is measuring gravity cleanly.
    static constexpr T expected_gravity_norm = constants::gravity<T>();

    /// @brief Minimum accepted accelerometer norm.
    /// @details Samples below this often indicate free-fall, severe vibration, or a broken sensor.
    static constexpr T accel_norm_min = static_cast<T>(8.0);

    /// @brief Maximum accepted accelerometer norm.
    /// @details Samples above this usually indicate impact, strong linear acceleration, or clipping.
    static constexpr T accel_norm_max = static_cast<T>(11.5);

    /// @brief Maximum accepted gyroscope magnitude in rad/s.
    static constexpr T gyro_norm_max = static_cast<T>(35.0);

    /// @brief Expected magnetic field norm in deployment-specific units after calibration.
    /// @details This acts as an optional bootstrap hint when the calibrated field strength is
    ///          already known in the application's chosen units.
    static constexpr T expected_mag_norm = static_cast<T>(50.0);

    /// @brief Minimum magnetic field norm used for optional bootstrap confidence.
    static constexpr T mag_norm_min = static_cast<T>(20.0);

    /// @brief Maximum magnetic field norm used for optional bootstrap confidence.
    static constexpr T mag_norm_max = static_cast<T>(70.0);

    /// @brief Allowed fractional deviation from the learned magnetic-field norm.
    static constexpr T mag_relative_norm_tolerance = static_cast<T>(0.35);

    /// @brief Learning rate for the online magnetic-field norm tracker.
    static constexpr T mag_reference_learning_alpha = static_cast<T>(0.05);

    /// @brief Maximum directional error tolerated before a trusted magnetometer update is rejected.
    static constexpr T mag_alignment_max_error_rad = static_cast<T>(2.35619449019);

    /// @brief Low-pass coefficient used to smooth gravity estimation from accelerometer data.
    /// @details 1.0 uses the current sample directly. Smaller values react more slowly but reject
    ///          vibration and impulse contamination better.
    static constexpr T gravity_filter_alpha = static_cast<T>(0.2);

    /// @brief Gyro norm threshold for stationary detection.
    static constexpr T stationary_gyro_norm_max = static_cast<T>(0.05);

    /// @brief Maximum allowed accelerometer norm error for stationary detection.
    static constexpr T stationary_accel_error_max = static_cast<T>(0.15);

    /// @brief Proportional gain applied to accelerometer correction in the Mahony backend.
    static constexpr T kp_accel = static_cast<T>(2.5);

    /// @brief Proportional gain applied to magnetometer correction in the Mahony backend.
    static constexpr T kp_mag = static_cast<T>(1.5);

    /// @brief Integral gain used to adapt gyro bias in the Mahony backend.
    static constexpr T ki_gyro_bias = static_cast<T>(0.05);

    /// @brief Heuristic drift-growth rate used while no absolute heading correction is available.
    static constexpr T drift_rate_without_heading_rad_per_s = static_cast<T>(0.02);

    /// @brief Drift magnitude above which confidence is heavily penalized.
    static constexpr T drift_confidence_limit_rad = static_cast<T>(0.35);

    /// @brief Attitude process noise used by the MEKF backend.
    static constexpr T q_attitude = static_cast<T>(1e-3);

    /// @brief Gyro-bias process noise used by the MEKF backend.
    static constexpr T q_gyro_bias = static_cast<T>(1e-5);

    /// @brief Accelerometer measurement variance used by the MEKF backend.
    static constexpr T r_accel = static_cast<T>(3e-2);

    /// @brief Magnetometer measurement variance used by the MEKF backend.
    static constexpr T r_mag = static_cast<T>(5e-2);

    /// @brief Normalized innovation squared gate for accelerometer updates in the MEKF backend.
    static constexpr T accel_nis_gate = static_cast<T>(7.81);

    /// @brief Normalized innovation squared gate for magnetometer updates in the MEKF backend.
    static constexpr T mag_nis_gate = static_cast<T>(7.81);

    /// @brief Body-frame axis that is considered the local down direction when the entity is level.
    static auto body_down_axis() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(-1)); }

    /// @brief Body-frame axis used to define heading in world coordinates.
    static auto body_heading_axis() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(1), T(0), T(0)); }

    /// @brief World-frame gravity direction.
    static auto world_gravity_direction() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(-1)); }

    /// @brief World-frame magnetic-reference direction used for heading alignment.
    static auto world_magnetic_reference() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(1), T(0), T(0)); }
};

/// @brief Default accelerometer-only configuration.
template<typename T>
struct DefaultAccelOnlyConfig : OrientationConfigBase<T>
{
    // cppcheck-suppress duplInheritedMember
    static constexpr SensorModel sensor_model = SensorModel::accel_only;
    // cppcheck-suppress duplInheritedMember
    static constexpr FusionBackend backend = FusionBackend::mahony;
    // cppcheck-suppress duplInheritedMember
    static constexpr bool estimate_gyro_bias = false;
    // cppcheck-suppress duplInheritedMember
    static constexpr bool enable_mag_correction = false;
};

/// @brief Default 6-axis IMU configuration using the Mahony backend.
template<typename T>
struct DefaultImu6MahonyConfig : OrientationConfigBase<T>
{
    // cppcheck-suppress duplInheritedMember
    static constexpr SensorModel sensor_model = SensorModel::imu6;
    // cppcheck-suppress duplInheritedMember
    static constexpr FusionBackend backend = FusionBackend::mahony;
    // cppcheck-suppress duplInheritedMember
    static constexpr bool enable_mag_correction = false;
};

/// @brief Default 6-axis IMU configuration using the optional MEKF backend.
template<typename T>
struct DefaultImu6EkfConfig : OrientationConfigBase<T>
{
    // cppcheck-suppress duplInheritedMember
    static constexpr SensorModel sensor_model = SensorModel::imu6;
    // cppcheck-suppress duplInheritedMember
    static constexpr FusionBackend backend = FusionBackend::mekf;
    // cppcheck-suppress duplInheritedMember
    static constexpr bool enable_mag_correction = false;
};

/// @brief Default 9-axis IMU configuration using the Mahony backend.
template<typename T>
struct DefaultImu9MahonyConfig : OrientationConfigBase<T>
{
    // cppcheck-suppress duplInheritedMember
    static constexpr SensorModel sensor_model = SensorModel::imu9;
    // cppcheck-suppress duplInheritedMember
    static constexpr FusionBackend backend = FusionBackend::mahony;
};

/// @brief Default 9-axis IMU configuration using the optional MEKF backend.
template<typename T>
struct DefaultImu9EkfConfig : OrientationConfigBase<T>
{
    // cppcheck-suppress duplInheritedMember
    static constexpr SensorModel sensor_model = SensorModel::imu9;
    // cppcheck-suppress duplInheritedMember
    static constexpr FusionBackend backend = FusionBackend::mekf;
};

/// @brief Shared compile-time policy knobs for relative-angle extraction.
template<typename T>
struct RelativeAngleConfigBase
{
    static_assert(std::is_floating_point<T>::value, "Relative-angle configs require a floating-point scalar type.");

    /// @brief Default scalar returned by compute().
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::shortest_3d_angle;

    /// @brief Allows compute_scalar(mode) to override the default scalar at runtime.
    static constexpr bool allow_runtime_output_override = true;

    /// @brief Allows heading-dependent outputs to be returned with drift metadata instead of being rejected.
    static constexpr bool allow_drift_with_quality_flags = true;

    /// @brief Applies a captured neutral reference pose before extracting scalar outputs.
    static constexpr bool apply_reference_pose = false;

    /// @brief Maximum tolerated timestamp skew between left and right entity estimates.
    static constexpr T max_pair_skew_s = static_cast<T>(0.03);

    /// @brief Maximum forward-prediction horizon used to align recent entity states in compute().
    static constexpr T max_alignment_horizon_s = static_cast<T>(0.05);

    /// @brief Confidence floor required for the relative result to be marked valid.
    static constexpr T min_confidence_to_publish = static_cast<T>(0.35);

    /// @brief Drift threshold above which a relative result is marked degraded.
    static constexpr T nominal_drift_limit_rad = static_cast<T>(0.0872664626);

    /// @brief Left-entity hinge axis used for twist and swing extraction.
    static auto hinge_axis_left() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(1)); }

    /// @brief Right-entity hinge axis used for diagnostics and alignment checks.
    static auto hinge_axis_right() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(1)); }
};

/// @brief Default relative-angle configuration.
template<typename T>
struct DefaultRelativeAngleConfig : RelativeAngleConfigBase<T>
{
};

namespace detail
{

template<typename Sample>
struct SampleTraits;

template<typename T>
struct SampleTraits<AccelSample<T>>
{
    static constexpr bool has_gyro = false;
    static constexpr bool has_mag = false;

    /// @brief Returns the sample timestamp in seconds.
    static auto timestamp(const AccelSample<T>& sample) noexcept -> T { return sample.timestamp_s; }
    /// @brief Returns the accelerometer vector.
    static auto accel(const AccelSample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.accel; }
    /// @brief Returns a zero angular-rate vector for accel-only samples.
    static auto gyro(const AccelSample<T>&) noexcept -> Vec<T, 3> { return Vec<T, 3>(); }
    /// @brief Returns a zero magnetic-field vector for accel-only samples.
    static auto mag(const AccelSample<T>&) noexcept -> Vec<T, 3> { return Vec<T, 3>(); }
};

template<typename T>
struct SampleTraits<Imu6Sample<T>>
{
    static constexpr bool has_gyro = true;
    static constexpr bool has_mag = false;

    /// @brief Returns the sample timestamp in seconds.
    static auto timestamp(const Imu6Sample<T>& sample) noexcept -> T { return sample.timestamp_s; }
    /// @brief Returns the accelerometer vector.
    static auto accel(const Imu6Sample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.accel; }
    /// @brief Returns the gyroscope vector.
    static auto gyro(const Imu6Sample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.gyro; }
    /// @brief Returns a zero magnetic-field vector for 6-axis samples.
    static auto mag(const Imu6Sample<T>&) noexcept -> Vec<T, 3> { return Vec<T, 3>(); }
};

template<typename T>
struct SampleTraits<Imu9Sample<T>>
{
    static constexpr bool has_gyro = true;
    static constexpr bool has_mag = true;

    /// @brief Returns the sample timestamp in seconds.
    static auto timestamp(const Imu9Sample<T>& sample) noexcept -> T { return sample.timestamp_s; }
    /// @brief Returns the accelerometer vector.
    static auto accel(const Imu9Sample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.accel; }
    /// @brief Returns the gyroscope vector.
    static auto gyro(const Imu9Sample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.gyro; }
    /// @brief Returns the magnetometer vector.
    static auto mag(const Imu9Sample<T>& sample) noexcept -> const Vec<T, 3>& { return sample.mag; }
};

template<typename T, SensorModel Model>
struct SampleType;

template<typename T>
struct SampleType<T, SensorModel::accel_only>
{
    using type = AccelSample<T>;
};

template<typename T>
struct SampleType<T, SensorModel::imu6>
{
    using type = Imu6Sample<T>;
};

template<typename T>
struct SampleType<T, SensorModel::imu9>
{
    using type = Imu9Sample<T>;
};

template<typename T, SensorModel Model>
using SampleTypeT = typename SampleType<T, Model>::type;

/// @brief Clamps a scalar to the closed unit interval [-1, 1].
template<typename T>
inline auto clamp_unit(T value) noexcept -> T
{
    return std::max(T(-1), std::min(T(1), value));
}

/// @brief Clamps a scalar to the probability-style interval [0, 1].
template<typename T>
inline auto saturate(T value) noexcept -> T
{
    return std::max(T(0), std::min(T(1), value));
}

/// @brief Wraps an angle into the interval [-pi, pi].
template<typename T>
inline auto wrap_pi(T angle) noexcept -> T
{
    const T pi = constants::pi<T>();
    const T two_pi = constants::two_pi<T>();
    while (angle > pi)
    {
        angle -= two_pi;
    }
    while (angle < -pi)
    {
        angle += two_pi;
    }
    return angle;
}

/// @brief Checks whether every component of a 3-vector is finite.
template<typename T>
inline auto is_finite(const Vec<T, 3>& value) noexcept -> bool
{
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

/// @brief Checks whether every component of a quaternion is finite.
template<typename T>
inline auto is_finite(const Quaternion<T>& value) noexcept -> bool
{
    return std::isfinite(value.w()) && std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

/// @brief Checks whether every component of a matrix is finite.
template<typename T, std::size_t Rows, std::size_t Cols>
inline auto is_finite(const Mat<T, Rows, Cols>& value) noexcept -> bool
{
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t col = 0; col < Cols; ++col)
        {
            if (!std::isfinite(value(row, col)))
            {
                return false;
            }
        }
    }
    return true;
}

/// @brief Reports whether a vector is finite and non-degenerate.
template<typename T>
inline auto is_usable_direction(const Vec<T, 3>& value) noexcept -> bool
{
    return is_finite(value) && value.length_squared() > std::numeric_limits<T>::epsilon();
}

/// @brief Normalizes a vector or returns a fallback when the input is degenerate.
template<typename T>
inline auto normalized_or(const Vec<T, 3>& value, const Vec<T, 3>& fallback) noexcept -> Vec<T, 3>
{
    const T length_sq = value.length_squared();
    if (!(length_sq > std::numeric_limits<T>::epsilon()))
    {
        return fallback;
    }
    return value / std::sqrt(length_sq);
}

/// @brief Computes the cosine between two directions or returns a fallback for degenerate inputs.
template<typename T>
inline auto direction_cosine(const Vec<T, 3>& lhs, const Vec<T, 3>& rhs, T fallback = T(1)) noexcept -> T
{
    if (!is_usable_direction(lhs) || !is_usable_direction(rhs))
    {
        return fallback;
    }
    return clamp_unit(normalized_or(lhs, rhs).dot(normalized_or(rhs, lhs)));
}

/// @brief Converts a small rotation vector into a quaternion increment.
template<typename T>
inline auto small_angle_quaternion(const Vec<T, 3>& delta) noexcept -> Quaternion<T>
{
    const T angle = delta.length();
    if (!(angle > std::numeric_limits<T>::epsilon()))
    {
        return Quaternion<T>::identity();
    }
    return Quaternion<T>::from_axis_angle(delta / angle, angle);
}

/// @brief Builds the skew-symmetric matrix associated with a 3-vector.
template<typename T>
inline auto skew(const Vec<T, 3>& value) noexcept -> Mat<T, 3, 3>
{
    return Mat<T, 3, 3>({T(0), -value[2], value[1], value[2], T(0), -value[0], -value[1], value[0], T(0)});
}

/// @brief Projects a vector onto the plane orthogonal to `normal`.
template<typename T>
inline auto project_onto_plane(const Vec<T, 3>& value, const Vec<T, 3>& normal) noexcept -> Vec<T, 3>
{
    const Vec<T, 3> unit_normal = normalized_or(normal, Vec<T, 3>(T(0), T(0), T(1)));
    return value - unit_normal * value.dot(unit_normal);
}

/// @brief Extracts the shortest unsigned rotation angle represented by a quaternion.
template<typename T>
inline auto shortest_angle_from_quaternion(const Quaternion<T>& q) noexcept -> T
{
    const Quaternion<T> normalized = q.normalized();
    const T w = std::abs(normalized.w());
    return T(2) * std::acos(clamp_unit(w));
}

/// @brief Extracts the twist component of a quaternion about a selected axis.
template<typename T>
inline auto twist_quaternion_about_axis(const Quaternion<T>& q, const Vec<T, 3>& axis) noexcept -> Quaternion<T>
{
    const Vec<T, 3> unit_axis = normalized_or(axis, Vec<T, 3>(T(0), T(0), T(1)));
    const Vec<T, 3> projected = unit_axis * q.vec().dot(unit_axis);
    Quaternion<T> twist(q.w(), projected[0], projected[1], projected[2]);
    if (!(twist.norm_squared() > std::numeric_limits<T>::epsilon()))
    {
        return Quaternion<T>::identity();
    }
    return twist.normalized();
}

/// @brief Extracts the signed twist angle of a quaternion about a selected axis.
template<typename T>
inline auto twist_angle_about_axis(const Quaternion<T>& q, const Vec<T, 3>& axis) noexcept -> T
{
    const Vec<T, 3> unit_axis = normalized_or(axis, Vec<T, 3>(T(0), T(0), T(1)));
    const Quaternion<T> twist = twist_quaternion_about_axis(q, unit_axis);
    const T unsigned_angle = T(2) * std::atan2(twist.vec().length(), std::abs(twist.w()));
    const T sign = twist.vec().dot(unit_axis) >= T(0) ? T(1) : T(-1);
    return wrap_pi(sign * unsigned_angle);
}

/// @brief Extracts the swing angle orthogonal to a selected twist axis.
template<typename T>
inline auto swing_angle_about_axis(const Quaternion<T>& q, const Vec<T, 3>& axis) noexcept -> T
{
    const Quaternion<T> twist = twist_quaternion_about_axis(q, axis);
    const Quaternion<T> swing = (q * twist.inverse_unit()).normalized();
    return shortest_angle_from_quaternion(swing);
}

/// @brief Initializes an orientation estimate from gravity alone.
template<typename T>
inline auto initialize_from_accel(const Vec<T, 3>& accel, const Vec<T, 3>& world_gravity) noexcept -> Quaternion<T>
{
    const Vec<T, 3> measured_gravity = normalized_or(accel, world_gravity);
    return Quaternion<T>::from_two_vectors(measured_gravity,
                                           normalized_or(world_gravity, Vec<T, 3>(T(0), T(0), T(-1))));
}

/// @brief Applies magnetic heading alignment to a tilt-only orientation estimate.
template<typename T>
inline auto apply_heading_alignment(const Quaternion<T>& tilt_orientation, const Vec<T, 3>& mag_body,
                                    const Vec<T, 3>& world_gravity,
                                    const Vec<T, 3>& world_mag) noexcept -> Quaternion<T>
{
    const Vec<T, 3> world_up = normalized_or(-world_gravity, Vec<T, 3>(T(0), T(0), T(1)));
    const Vec<T, 3> predicted_mag_world = tilt_orientation.rotate(normalized_or(mag_body, world_mag));
    const Vec<T, 3> predicted_heading = project_onto_plane(predicted_mag_world, world_up);
    const Vec<T, 3> reference_heading =
        project_onto_plane(normalized_or(world_mag, Vec<T, 3>(T(1), T(0), T(0))), world_up);
    if (!(predicted_heading.length_squared() > std::numeric_limits<T>::epsilon()) ||
        !(reference_heading.length_squared() > std::numeric_limits<T>::epsilon()))
    {
        return tilt_orientation;
    }

    const T delta = normalized_or(predicted_heading, reference_heading)
                        .signed_angle(normalized_or(reference_heading, predicted_heading), world_up);
    return (Quaternion<T>::from_axis_angle(world_up, delta) * tilt_orientation).normalized();
}

/// @brief Initializes an orientation estimate from gravity and magnetic reference vectors.
template<typename T>
inline auto initialize_from_accel_and_mag(const Vec<T, 3>& accel, const Vec<T, 3>& mag, const Vec<T, 3>& world_gravity,
                                          const Vec<T, 3>& world_mag) noexcept -> Quaternion<T>
{
    const Quaternion<T> tilt_only = initialize_from_accel(accel, world_gravity);
    return apply_heading_alignment(tilt_only, mag, world_gravity, world_mag);
}

/// @brief Checks whether a configuration selects a supported backend and sensor model pair.
template<typename Config>
constexpr auto backend_is_compatible() noexcept -> bool
{
    if constexpr (Config::sensor_model == SensorModel::accel_only)
    {
        return Config::backend != FusionBackend::mekf && !Config::estimate_gyro_bias;
    }
    if constexpr (Config::sensor_model == SensorModel::imu6)
    {
        return !Config::enable_mag_correction;
    }
    return true;
}

/// @brief Reports whether a configuration can produce absolute heading observability.
template<typename Config>
constexpr auto has_absolute_heading_capability() noexcept -> bool
{
    return Config::sensor_model == SensorModel::imu9 && Config::enable_mag_correction;
}

/// @brief Reports whether a scalar output depends on heading observability.
constexpr auto output_requires_heading(PrimaryScalarOutput output) noexcept -> bool
{
    switch (output)
    {
    case PrimaryScalarOutput::tilt_angle:
        return false;
    case PrimaryScalarOutput::shortest_3d_angle:
    case PrimaryScalarOutput::heading_delta:
    case PrimaryScalarOutput::hinge_twist:
    case PrimaryScalarOutput::swing_angle:
    case PrimaryScalarOutput::roll_delta:
    case PrimaryScalarOutput::pitch_delta:
    case PrimaryScalarOutput::yaw_delta:
    default:
        return true;
    }
}

/// @brief Maps observability values to an integer rank for comparisons.
inline auto observability_rank(Observability observability) noexcept -> int
{
    switch (observability)
    {
    case Observability::none:
        return 0;
    case Observability::tilt_only:
        return 1;
    case Observability::heading_with_drift:
        return 2;
    case Observability::full_3d:
        return 3;
    }
    return 0;
}

/// @brief Combines two entity observability levels into the pairwise minimum.
inline auto combined_observability(Observability left, Observability right) noexcept -> Observability
{
    const int rank = std::min(observability_rank(left), observability_rank(right));
    switch (rank)
    {
    case 3:
        return Observability::full_3d;
    case 2:
        return Observability::heading_with_drift;
    case 1:
        return Observability::tilt_only;
    default:
        return Observability::none;
    }
}

/// @brief Converts confidence and status flags into a coarse quality grade.
template<typename T>
inline auto quality_from_confidence(T confidence, StatusFlags flags) noexcept -> SolutionQuality
{
    if (confidence < static_cast<T>(0.15))
    {
        return SolutionQuality::invalid;
    }
    if (confidence < static_cast<T>(0.5) || flags != 0U)
    {
        return SolutionQuality::degraded;
    }
    if (confidence < static_cast<T>(0.85))
    {
        return SolutionQuality::usable;
    }
    return SolutionQuality::nominal;
}

/// @brief Scores a norm against an expected range-centered reference value.
template<typename T>
inline auto confidence_from_norm(T value, T expected, T min_value, T max_value) noexcept -> T
{
    if (!(value >= min_value && value <= max_value))
    {
        return T(0);
    }

    const T upper_span = std::max(std::abs(max_value - expected), std::abs(expected - min_value));
    if (!(upper_span > std::numeric_limits<T>::epsilon()))
    {
        return T(1);
    }

    return saturate(T(1) - (std::abs(value - expected) / upper_span));
}

template<typename T>
struct MagMeasurementEvaluation
{
    bool available{};
    bool valid{};
    T confidence{};
    T norm{};
};

/// @brief Builds an isotropic 3x3 covariance matrix.
template<typename T>
inline auto diagonal_covariance(T variance) noexcept -> Mat<T, 3, 3>
{
    return Mat<T, 3, 3>::identity() * variance;
}

/// @brief Builds the 6x6 process covariance used by the MEKF propagation step.
template<typename T>
inline auto diagonal_process_covariance(T attitude_noise, T bias_noise, T dt) noexcept -> Mat<T, 6, 6>
{
    Mat<T, 6, 6> q = Mat<T, 6, 6>::zero();
    const T attitude_term = attitude_noise * std::max(dt, T(0));
    const T bias_term = bias_noise * std::max(dt, T(0));
    for (std::size_t i = 0; i < 3; ++i)
    {
        q(i, i) = attitude_term;
        q(i + 3, i + 3) = bias_term;
    }
    return q;
}

/// @brief Checks whether a 3x3 matrix is invertible by a simple determinant threshold.
template<typename T>
inline auto matrix_determinant_is_invertible(const Mat<T, 3, 3>& matrix) noexcept -> bool
{
    return std::abs(matrix.determinant()) > std::numeric_limits<T>::epsilon();
}

}  // namespace detail

template<typename T, typename Config>
class OrientationEstimator
{
public:
    using Sample = detail::SampleTypeT<T, Config::sensor_model>;

    /// @brief Creates an estimator with default runtime calibration.
    OrientationEstimator() noexcept : config_valid_(validate_configuration()) { refresh_calibration_flags(); }

    /// @brief Creates an estimator with caller-supplied runtime calibration.
    explicit OrientationEstimator(const SensorCalibration<T>& calibration) noexcept
        : calibration_(calibration), config_valid_(validate_configuration())
    {
        refresh_calibration_flags();
    }

    static_assert(std::is_floating_point<T>::value, "OrientationEstimator requires a floating-point scalar type.");
    static_assert(detail::backend_is_compatible<Config>(),
                  "The selected sensor model and backend combination is not supported by the sensor-fusion API.");

    /// @brief Updates the estimator with a new sample.
    void update(const Sample& sample) noexcept
    {
        estimate_ = OrientationEstimate<T>();
        estimate_.orientation = orientation_;
        if (!config_valid_)
        {
            reject_current_sample(StatusFlag::configuration_invalid);
            return;
        }
        if (!calibration_valid_)
        {
            reject_current_sample(StatusFlag::calibration_invalid);
            return;
        }

        const T sample_timestamp_s = detail::SampleTraits<Sample>::timestamp(sample);
        if (!std::isfinite(sample_timestamp_s))
        {
            reject_current_sample(StatusFlag::sample_time_invalid);
            return;
        }
        estimate_.timestamp_s = sample_timestamp_s;

        T dt = T(0);
        bool accept_sample_timestamp = true;
        if (have_timestamp_)
        {
            dt = estimate_.timestamp_s - last_timestamp_s_;
            if (dt < Config::min_dt_s)
            {
                set_flag(estimate_.flags, StatusFlag::sample_time_invalid);
                accept_sample_timestamp = false;
                dt = T(0);
            }
            else if (dt > Config::max_dt_s)
            {
                set_flag(estimate_.flags, StatusFlag::sample_time_invalid);
                dt = T(0);
            }
        }
        if (!have_timestamp_ || accept_sample_timestamp)
        {
            last_timestamp_s_ = estimate_.timestamp_s;
            have_timestamp_ = true;
        }

        const Vec<T, 3> accel = calibrate_accel(detail::SampleTraits<Sample>::accel(sample));
        const Vec<T, 3> gyro = calibrate_gyro(detail::SampleTraits<Sample>::gyro(sample));
        const Vec<T, 3> mag = calibrate_mag(detail::SampleTraits<Sample>::mag(sample));

        if constexpr (Config::sensor_model == SensorModel::accel_only)
        {
            update_accel_only(accel);
        }
        else if constexpr (Config::backend == FusionBackend::mahony)
        {
            update_mahony(accel, gyro, mag, dt);
        }
        else
        {
            update_mekf(accel, gyro, mag, dt);
        }

        estimate_.orientation = orientation_;
        estimate_.timestamp_s = last_timestamp_s_;
        estimate_.heading_referenced = heading_referenced_;
        estimate_.estimated_drift_rad = estimated_drift_rad_;
        estimate_.observability = current_observability();
        estimate_.confidence = current_confidence(last_accel_confidence_, last_mag_confidence_);
        estimate_.quality = detail::quality_from_confidence(estimate_.confidence, estimate_.flags);
        estimate_.valid = initialized_ && estimate_.quality != SolutionQuality::invalid;
    }

    /// @brief Resets the internal state.
    void reset() noexcept
    {
        orientation_ = Quaternion<T>::identity();
        gyro_bias_estimate_ = Vec<T, 3>();
        filtered_accel_ = Config::world_gravity_direction();
        covariance_ = Mat<T, 6, 6>::identity() * static_cast<T>(0.1);
        estimate_ = OrientationEstimate<T>();
        initialized_ = false;
        have_timestamp_ = false;
        heading_referenced_ = false;
        estimated_drift_rad_ = T(0);
        last_accel_confidence_ = T(0);
        last_mag_confidence_ = T(0);
        mag_reference_norm_ = T(0);
        have_mag_reference_norm_ = false;
        last_body_rate_rad_per_s_ = Vec<T, 3>();
        can_predict_forward_ = false;
    }

    /// @brief Sets runtime sensor calibration and resets estimator state.
    void set_calibration(const SensorCalibration<T>& calibration) noexcept
    {
        calibration_ = calibration;
        refresh_calibration_flags();
        reset();
    }

    /// @brief Returns the latest estimate.
    [[nodiscard]] auto estimate() const noexcept -> const OrientationEstimate<T>& { return estimate_; }

    /// @brief Returns the current orientation quaternion.
    [[nodiscard]] auto orientation() const noexcept -> Quaternion<T> { return orientation_; }

    /// @brief Predicts the orientation forward to a later timestamp using the latest valid body rate.
    [[nodiscard]] auto predict_orientation(T target_timestamp_s, T max_horizon_s) const noexcept -> Quaternion<T>
    {
        if (!initialized_ || !can_predict_forward_)
        {
            return orientation_;
        }

        const T dt = target_timestamp_s - last_timestamp_s_;
        if (!(dt > T(0)) || dt > max_horizon_s)
        {
            return orientation_;
        }

        return (orientation_ * detail::small_angle_quaternion(last_body_rate_rad_per_s_ * dt)).normalized();
    }

    /// @brief Returns the current gyro-bias estimate.
    [[nodiscard]] auto gyro_bias_estimate() const noexcept -> const Vec<T, 3>& { return gyro_bias_estimate_; }

private:
    SensorCalibration<T> calibration_{};
    Quaternion<T> orientation_ = Quaternion<T>::identity();
    Vec<T, 3> gyro_bias_estimate_{};
    Vec<T, 3> filtered_accel_ = Config::world_gravity_direction();
    Mat<T, 6, 6> covariance_ = Mat<T, 6, 6>::identity() * static_cast<T>(0.1);
    OrientationEstimate<T> estimate_{};
    bool initialized_{};
    bool have_timestamp_{};
    bool heading_referenced_{};
    bool sensor_to_body_is_identity_{true};
    bool accel_bias_is_zero_{true};
    bool accel_scale_is_identity_{true};
    bool gyro_bias_is_zero_{true};
    bool gyro_scale_is_identity_{true};
    bool mag_bias_is_zero_{true};
    bool mag_soft_iron_is_identity_{true};
    bool accel_calibration_passthrough_{true};
    bool gyro_calibration_passthrough_{true};
    bool mag_calibration_passthrough_{true};
    bool have_mag_reference_norm_{};
    bool can_predict_forward_{};
    bool calibration_valid_{true};
    bool config_valid_{true};
    T last_timestamp_s_{};
    T estimated_drift_rad_{};
    T last_accel_confidence_{};
    T last_mag_confidence_{};
    T mag_reference_norm_{};
    Vec<T, 3> last_body_rate_rad_per_s_{};

    /// @brief Recomputes calibration fast-path flags from the current runtime calibration.
    void refresh_calibration_flags() noexcept
    {
        calibration_valid_ = normalize_and_validate_calibration();
        if (!calibration_valid_)
        {
            sensor_to_body_is_identity_ = false;
            accel_bias_is_zero_ = false;
            accel_scale_is_identity_ = false;
            gyro_bias_is_zero_ = false;
            gyro_scale_is_identity_ = false;
            mag_bias_is_zero_ = false;
            mag_soft_iron_is_identity_ = false;
            accel_calibration_passthrough_ = false;
            gyro_calibration_passthrough_ = false;
            mag_calibration_passthrough_ = false;
            return;
        }

        sensor_to_body_is_identity_ = calibration_.sensor_to_body.is_identity();
        accel_bias_is_zero_ = calibration_.accel_bias.is_zero();
        accel_scale_is_identity_ = calibration_.accel_scale.is_one();
        gyro_bias_is_zero_ = calibration_.gyro_bias.is_zero();
        gyro_scale_is_identity_ = calibration_.gyro_scale.is_one();
        mag_bias_is_zero_ = calibration_.mag_bias.is_zero();
        mag_soft_iron_is_identity_ = calibration_.mag_soft_iron.is_identity();

        accel_calibration_passthrough_ = sensor_to_body_is_identity_ && accel_bias_is_zero_ && accel_scale_is_identity_;
        gyro_calibration_passthrough_ = sensor_to_body_is_identity_ && gyro_bias_is_zero_ && gyro_scale_is_identity_;
        mag_calibration_passthrough_ = sensor_to_body_is_identity_ && mag_bias_is_zero_ && mag_soft_iron_is_identity_;
    }

    /// @brief Rejects a malformed sample while preserving the last valid state.
    void reject_current_sample(StatusFlag reason) noexcept
    {
        set_flag(estimate_.flags, reason);
        if (!initialized_)
        {
            set_flag(estimate_.flags, StatusFlag::startup_not_initialized);
        }
        estimate_.orientation = orientation_;
        estimate_.timestamp_s = have_timestamp_ ? last_timestamp_s_ : T(0);
        estimate_.heading_referenced = heading_referenced_;
        estimate_.estimated_drift_rad = estimated_drift_rad_;
        estimate_.observability = current_observability();
        estimate_.confidence = T(0);
        estimate_.quality = SolutionQuality::invalid;
        estimate_.valid = false;
        last_accel_confidence_ = T(0);
        last_mag_confidence_ = T(0);
        update_prediction_rate(Vec<T, 3>(), false);
    }

    /// @brief Returns whether a scalar configuration value is finite.
    [[nodiscard]] static auto scalar_is_finite(T value) noexcept -> bool { return std::isfinite(value); }

    /// @brief Returns whether a configuration axis is finite and non-degenerate.
    [[nodiscard]] static auto config_axis_is_valid(const Vec<T, 3>& axis) noexcept -> bool
    {
        return detail::is_usable_direction(axis);
    }

    /// @brief Validates the estimator's compile-time policy values.
    [[nodiscard]] static auto validate_configuration() noexcept -> bool
    {
        if (!config_axis_is_valid(Config::body_down_axis()) || !config_axis_is_valid(Config::body_heading_axis()) ||
            !config_axis_is_valid(Config::world_gravity_direction()))
        {
            return false;
        }

        if (!scalar_is_finite(Config::min_dt_s) || !scalar_is_finite(Config::max_dt_s) ||
            !scalar_is_finite(Config::max_sample_age_s) || !scalar_is_finite(Config::expected_gravity_norm) ||
            !scalar_is_finite(Config::accel_norm_min) || !scalar_is_finite(Config::accel_norm_max) ||
            !scalar_is_finite(Config::gyro_norm_max) || !scalar_is_finite(Config::gravity_filter_alpha) ||
            !scalar_is_finite(Config::stationary_gyro_norm_max) ||
            !scalar_is_finite(Config::stationary_accel_error_max) ||
            !scalar_is_finite(Config::drift_rate_without_heading_rad_per_s) ||
            !scalar_is_finite(Config::drift_confidence_limit_rad))
        {
            return false;
        }

        if (!(Config::min_dt_s > T(0)) || Config::max_dt_s < Config::min_dt_s || Config::max_sample_age_s < T(0) ||
            !(Config::expected_gravity_norm > T(0)) || !(Config::accel_norm_min > T(0)) ||
            Config::accel_norm_max < Config::accel_norm_min || !(Config::gyro_norm_max > T(0)) ||
            Config::gravity_filter_alpha < T(0) || Config::gravity_filter_alpha > T(1) ||
            Config::stationary_gyro_norm_max < T(0) || Config::stationary_accel_error_max < T(0) ||
            Config::drift_rate_without_heading_rad_per_s < T(0) || !(Config::drift_confidence_limit_rad > T(0)))
        {
            return false;
        }

        if constexpr (Config::sensor_model == SensorModel::imu9 && Config::enable_mag_correction)
        {
            if (!config_axis_is_valid(Config::world_magnetic_reference()) ||
                !scalar_is_finite(Config::expected_mag_norm) || !scalar_is_finite(Config::mag_norm_min) ||
                !scalar_is_finite(Config::mag_norm_max) || !scalar_is_finite(Config::mag_relative_norm_tolerance) ||
                !scalar_is_finite(Config::mag_reference_learning_alpha) ||
                !scalar_is_finite(Config::mag_alignment_max_error_rad))
            {
                return false;
            }

            if (!(Config::expected_mag_norm > T(0)) || !(Config::mag_norm_min > T(0)) ||
                Config::mag_norm_max < Config::mag_norm_min || Config::mag_relative_norm_tolerance < T(0) ||
                Config::mag_reference_learning_alpha < T(0) || Config::mag_reference_learning_alpha > T(1) ||
                Config::mag_alignment_max_error_rad < T(0))
            {
                return false;
            }
        }

        if constexpr (Config::backend == FusionBackend::mahony)
        {
            if (!scalar_is_finite(Config::kp_accel) || !scalar_is_finite(Config::kp_mag) ||
                !scalar_is_finite(Config::ki_gyro_bias))
            {
                return false;
            }

            return Config::kp_accel >= T(0) && Config::kp_mag >= T(0) && Config::ki_gyro_bias >= T(0);
        }

        if (!scalar_is_finite(Config::q_attitude) || !scalar_is_finite(Config::q_gyro_bias) ||
            !scalar_is_finite(Config::r_accel) || !scalar_is_finite(Config::r_mag) ||
            !scalar_is_finite(Config::accel_nis_gate) || !scalar_is_finite(Config::mag_nis_gate))
        {
            return false;
        }

        return Config::q_attitude > T(0) && Config::q_gyro_bias >= T(0) && Config::r_accel > T(0) &&
               Config::r_mag > T(0) && Config::accel_nis_gate > T(0) && Config::mag_nis_gate > T(0);
    }

    /// @brief Normalizes and validates runtime calibration values before use.
    auto normalize_and_validate_calibration() noexcept -> bool
    {
        if (!detail::is_finite(calibration_.sensor_to_body) || !detail::is_finite(calibration_.accel_bias) ||
            !detail::is_finite(calibration_.accel_scale) || !detail::is_finite(calibration_.gyro_bias) ||
            !detail::is_finite(calibration_.gyro_scale) || !detail::is_finite(calibration_.mag_bias) ||
            !detail::is_finite(calibration_.mag_soft_iron))
        {
            return false;
        }

        const T quaternion_norm_sq = calibration_.sensor_to_body.norm_squared();
        if (!std::isfinite(quaternion_norm_sq) || !(quaternion_norm_sq > std::numeric_limits<T>::epsilon()))
        {
            return false;
        }
        calibration_.sensor_to_body = calibration_.sensor_to_body.normalized();

        for (std::size_t i = 0; i < 3; ++i)
        {
            if (!(std::abs(calibration_.accel_scale[i]) > std::numeric_limits<T>::epsilon()) ||
                !(std::abs(calibration_.gyro_scale[i]) > std::numeric_limits<T>::epsilon()))
            {
                return false;
            }
        }

        return detail::matrix_determinant_is_invertible(calibration_.mag_soft_iron);
    }

    /// @brief Updates the cached body-rate used for short forward prediction.
    void update_prediction_rate(const Vec<T, 3>& body_rate_rad_per_s, bool valid) noexcept
    {
        if (valid && detail::is_finite(body_rate_rad_per_s))
        {
            last_body_rate_rad_per_s_ = body_rate_rad_per_s;
            can_predict_forward_ = true;
            return;
        }

        last_body_rate_rad_per_s_ = Vec<T, 3>();
        can_predict_forward_ = false;
    }

    /// @brief Checks whether a magnetometer sample is available under the current policy.
    auto mag_is_available(const Vec<T, 3>& mag) const noexcept -> bool
    {
        if constexpr (Config::sensor_model != SensorModel::imu9 || !Config::enable_mag_correction)
        {
            return false;
        }

        return detail::is_finite(mag) && std::sqrt(mag.length_squared()) > std::numeric_limits<T>::epsilon();
    }

    /// @brief Scores magnetic-field strength against the learned or configured reference norm.
    auto mag_strength_confidence(T norm) const noexcept -> T
    {
        if (!(norm > std::numeric_limits<T>::epsilon()))
        {
            return T(0);
        }

        if (have_mag_reference_norm_ && mag_reference_norm_ > std::numeric_limits<T>::epsilon())
        {
            const T tolerance = std::max(Config::mag_relative_norm_tolerance, static_cast<T>(0.05));
            const T deviation = std::abs(norm - mag_reference_norm_) / mag_reference_norm_;
            if (deviation > tolerance)
            {
                return T(0);
            }
            return detail::saturate(T(1) - (deviation / tolerance));
        }

        if (Config::expected_mag_norm > std::numeric_limits<T>::epsilon() &&
            Config::mag_norm_max > Config::mag_norm_min)
        {
            const T expected = Config::expected_mag_norm;
            const T ratio = std::max(norm / expected, expected / norm);
            if (ratio > static_cast<T>(64))
            {
                return T(0);
            }
            if (norm >= Config::mag_norm_min && norm <= Config::mag_norm_max)
            {
                return detail::confidence_from_norm(norm, expected, Config::mag_norm_min, Config::mag_norm_max);
            }
            return static_cast<T>(0.5);
        }

        return T(1);
    }

    /// @brief Scores magnetic direction consistency against the current heading estimate.
    auto mag_alignment_confidence(const Vec<T, 3>& mag) const noexcept -> T
    {
        if (!initialized_ || !heading_referenced_)
        {
            return T(1);
        }

        const T max_error = Config::mag_alignment_max_error_rad;
        if (!(max_error > std::numeric_limits<T>::epsilon()))
        {
            return T(1);
        }

        const Vec<T, 3> expected_mag_body = orientation_.rotate_inverse(
            detail::normalized_or(Config::world_magnetic_reference(), Vec<T, 3>(T(1), T(0), T(0))));
        const Vec<T, 3> measured_mag_body = detail::normalized_or(mag, expected_mag_body);
        const T cosine = detail::clamp_unit(measured_mag_body.dot(expected_mag_body));
        const T angle = std::acos(cosine);
        if (angle > max_error)
        {
            return T(0);
        }

        return detail::saturate(T(1) - (angle / max_error));
    }

    /// @brief Evaluates magnetometer availability, confidence, and validity in one pass.
    auto evaluate_mag_measurement(const Vec<T, 3>& mag) const noexcept -> detail::MagMeasurementEvaluation<T>
    {
        detail::MagMeasurementEvaluation<T> evaluation;
        evaluation.available = mag_is_available(mag);
        if (!evaluation.available)
        {
            return evaluation;
        }

        evaluation.norm = std::sqrt(mag.length_squared());
        const T strength_confidence = mag_strength_confidence(evaluation.norm);
        const T alignment_confidence = mag_alignment_confidence(mag);
        evaluation.confidence = strength_confidence * alignment_confidence;
        evaluation.valid = evaluation.confidence > T(0);
        return evaluation;
    }

    /// @brief Updates the learned magnetic-field norm reference from a valid sample.
    void learn_mag_reference(T norm) noexcept
    {
        if (!(norm > std::numeric_limits<T>::epsilon()))
        {
            return;
        }

        if (!have_mag_reference_norm_)
        {
            mag_reference_norm_ = norm;
            have_mag_reference_norm_ = true;
            return;
        }

        const T alpha = detail::saturate(Config::mag_reference_learning_alpha);
        mag_reference_norm_ = mag_reference_norm_ * (T(1) - alpha) + norm * alpha;
    }

    /// @brief Applies runtime accelerometer calibration and mounting compensation.
    auto calibrate_accel(const Vec<T, 3>& raw) const noexcept -> Vec<T, 3>
    {
        if (accel_calibration_passthrough_)
        {
            return raw;
        }

        Vec<T, 3> corrected = raw;
        if (!accel_bias_is_zero_)
        {
            corrected -= calibration_.accel_bias;
        }
        if (!accel_scale_is_identity_)
        {
            corrected = corrected.hadamard(calibration_.accel_scale);
        }
        return sensor_to_body_is_identity_ ? corrected : calibration_.sensor_to_body.rotate(corrected);
    }

    /// @brief Applies runtime gyroscope calibration and mounting compensation.
    auto calibrate_gyro(const Vec<T, 3>& raw) const noexcept -> Vec<T, 3>
    {
        if (gyro_calibration_passthrough_)
        {
            return raw;
        }

        Vec<T, 3> corrected = raw;
        if (!gyro_bias_is_zero_)
        {
            corrected -= calibration_.gyro_bias;
        }
        if (!gyro_scale_is_identity_)
        {
            corrected = corrected.hadamard(calibration_.gyro_scale);
        }
        return sensor_to_body_is_identity_ ? corrected : calibration_.sensor_to_body.rotate(corrected);
    }

    /// @brief Applies runtime magnetometer calibration and mounting compensation.
    auto calibrate_mag(const Vec<T, 3>& raw) const noexcept -> Vec<T, 3>
    {
        if (mag_calibration_passthrough_)
        {
            return raw;
        }

        Vec<T, 3> corrected = raw;
        if (!mag_bias_is_zero_)
        {
            corrected -= calibration_.mag_bias;
        }
        if (!mag_soft_iron_is_identity_)
        {
            corrected = calibration_.mag_soft_iron * corrected;
        }
        return sensor_to_body_is_identity_ ? corrected : calibration_.sensor_to_body.rotate(corrected);
    }

    /// @brief Checks whether an accelerometer sample is finite and within the configured norm window.
    auto accel_is_valid(const Vec<T, 3>& accel) const noexcept -> bool
    {
        if (!detail::is_finite(accel))
        {
            return false;
        }
        const T norm = std::sqrt(accel.length_squared());
        return norm >= Config::accel_norm_min && norm <= Config::accel_norm_max;
    }

    /// @brief Checks whether a gyroscope sample is finite and below the configured saturation limit.
    auto gyro_is_valid(const Vec<T, 3>& gyro) const noexcept -> bool
    {
        return detail::is_finite(gyro) && std::sqrt(gyro.length_squared()) <= Config::gyro_norm_max;
    }

    /// @brief Checks whether a magnetometer sample passes the combined availability and confidence gates.
    auto mag_is_valid(const Vec<T, 3>& mag) const noexcept -> bool { return evaluate_mag_measurement(mag).valid; }

    /// @brief Detects whether the entity is stationary enough to adapt gyro bias safely.
    auto is_stationary(const Vec<T, 3>& accel, const Vec<T, 3>& gyro) const noexcept -> bool
    {
        const T accel_norm = std::sqrt(accel.length_squared());
        const T gyro_norm = std::sqrt(gyro.length_squared());
        return std::abs(accel_norm - Config::expected_gravity_norm) <= Config::stationary_accel_error_max &&
               gyro_norm <= Config::stationary_gyro_norm_max;
    }

    /// @brief Initializes state from the first valid gravity or gravity-plus-magnetic measurement.
    void initialize_from_measurement(const Vec<T, 3>& accel, const Vec<T, 3>& mag, bool mag_valid) noexcept
    {
        if (initialized_)
        {
            return;
        }

        if (!accel_is_valid(accel))
        {
            set_flag(estimate_.flags, StatusFlag::startup_not_initialized);
            return;
        }

        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            if (mag_valid && Config::enable_mag_correction)
            {
                orientation_ = detail::initialize_from_accel_and_mag(accel, mag, Config::world_gravity_direction(),
                                                                     Config::world_magnetic_reference());
                heading_referenced_ = true;
                learn_mag_reference(std::sqrt(mag.length_squared()));
            }
            else
            {
                orientation_ = detail::initialize_from_accel(accel, Config::world_gravity_direction());
                heading_referenced_ = false;
            }
        }
        else
        {
            orientation_ = detail::initialize_from_accel(accel, Config::world_gravity_direction());
            heading_referenced_ = false;
        }

        filtered_accel_ = accel;
        initialized_ = true;
    }

    /// @brief Updates heading observability and drift bookkeeping after one measurement step.
    void update_heading_state(bool has_absolute_heading, T dt) noexcept
    {
        if constexpr (Config::sensor_model == SensorModel::accel_only)
        {
            heading_referenced_ = false;
            estimated_drift_rad_ = T(0);
            return;
        }

        if (has_absolute_heading)
        {
            heading_referenced_ = true;
            if (dt > T(0))
            {
                estimated_drift_rad_ =
                    std::max(T(0), estimated_drift_rad_ - Config::drift_rate_without_heading_rad_per_s * dt);
            }
            return;
        }

        heading_referenced_ = false;
        if (dt > T(0))
        {
            estimated_drift_rad_ += Config::drift_rate_without_heading_rad_per_s * dt;
        }
        set_flag(estimate_.flags, StatusFlag::propagation_only);
    }

    /// @brief Applies a measurement-only correction when gyro propagation is unavailable.
    void apply_measurement_only_update(const Vec<T, 3>& accel, const Vec<T, 3>& mag, bool accel_valid, bool mag_valid,
                                       T dt) noexcept
    {
        if constexpr (Config::sensor_model != SensorModel::accel_only)
        {
            set_flag(estimate_.flags, StatusFlag::gyro_rejected);
        }

        if (!accel_valid)
        {
            update_heading_state(false, dt);
            return;
        }

        Quaternion<T> target = detail::initialize_from_accel(accel, Config::world_gravity_direction());
        T blend =
            std::max(static_cast<T>(0.2), std::max(last_accel_confidence_, static_cast<T>(0.25)) * static_cast<T>(0.6));
        bool has_absolute_heading = false;

        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            if (mag_valid && Config::enable_mag_correction)
            {
                target = detail::initialize_from_accel_and_mag(accel, mag, Config::world_gravity_direction(),
                                                               Config::world_magnetic_reference());
                blend = std::max(blend, std::max(last_mag_confidence_, static_cast<T>(0.25)) * static_cast<T>(0.6));
                learn_mag_reference(std::sqrt(mag.length_squared()));
                has_absolute_heading = true;
            }
        }

        orientation_ = initialized_ ? orientation_.slerp(target, detail::saturate(blend)).normalized() : target;
        filtered_accel_ = accel;
        initialized_ = true;
        update_heading_state(has_absolute_heading, dt);
    }

    /// @brief Processes one accelerometer-only update path.
    void update_accel_only(const Vec<T, 3>& accel) noexcept
    {
        if (!accel_is_valid(accel))
        {
            set_flag(estimate_.flags, StatusFlag::accel_rejected);
            set_flag(estimate_.flags, StatusFlag::startup_not_initialized);
            last_accel_confidence_ = T(0);
            last_mag_confidence_ = T(0);
            update_prediction_rate(Vec<T, 3>(), false);
            return;
        }

        const T alpha = detail::saturate(Config::gravity_filter_alpha);
        filtered_accel_ = initialized_ ? (filtered_accel_ * (T(1) - alpha) + accel * alpha) : accel;
        orientation_ = detail::initialize_from_accel(filtered_accel_, Config::world_gravity_direction());
        initialized_ = true;
        heading_referenced_ = false;
        estimated_drift_rad_ = T(0);
        last_accel_confidence_ = detail::confidence_from_norm(accel.length(), Config::expected_gravity_norm,
                                                              Config::accel_norm_min, Config::accel_norm_max);
        last_mag_confidence_ = T(0);
        update_prediction_rate(Vec<T, 3>(), false);
    }

    /// @brief Processes one update using the Mahony-style correction backend.
    void update_mahony(const Vec<T, 3>& accel, const Vec<T, 3>& gyro, const Vec<T, 3>& mag, T dt) noexcept
    {
        const bool accel_valid = accel_is_valid(accel);
        const bool gyro_valid = gyro_is_valid(gyro);
        const auto mag_measurement = evaluate_mag_measurement(mag);
        const bool mag_available = mag_measurement.available;
        const bool initialized_before_update = initialized_;

        initialize_from_measurement(accel, mag, mag_measurement.valid);
        if (!initialized_)
        {
            last_accel_confidence_ = T(0);
            last_mag_confidence_ = T(0);
            update_prediction_rate(Vec<T, 3>(), false);
            return;
        }

        const T alpha = detail::saturate(Config::gravity_filter_alpha);
        if (accel_valid)
        {
            filtered_accel_ = filtered_accel_ * (T(1) - alpha) + accel * alpha;
            last_accel_confidence_ = detail::confidence_from_norm(accel.length(), Config::expected_gravity_norm,
                                                                  Config::accel_norm_min, Config::accel_norm_max);
        }
        else
        {
            set_flag(estimate_.flags, StatusFlag::accel_rejected);
            last_accel_confidence_ = T(0);
        }

        bool mag_valid = false;
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            mag_valid = mag_measurement.valid;
            if (mag_valid)
            {
                last_mag_confidence_ = mag_measurement.confidence;
                if (initialized_before_update)
                {
                    learn_mag_reference(mag_measurement.norm);
                }
            }
            else if (mag_measurement.available && Config::enable_mag_correction)
            {
                set_flag(estimate_.flags, StatusFlag::mag_rejected);
                last_mag_confidence_ = T(0);
            }
            else
            {
                last_mag_confidence_ = T(0);
            }
        }
        else
        {
            last_mag_confidence_ = T(0);
        }

        if (!(dt > T(0)) || !gyro_valid)
        {
            update_prediction_rate(gyro - gyro_bias_estimate_, gyro_valid);
            apply_measurement_only_update(accel, mag, accel_valid, mag_valid, dt);
            return;
        }

        Vec<T, 3> correction{};
        if (accel_valid && Config::enable_accel_correction)
        {
            const Vec<T, 3> expected_gravity_body = orientation_.rotate_inverse(
                detail::normalized_or(Config::world_gravity_direction(), Vec<T, 3>(T(0), T(0), T(-1))));
            const Vec<T, 3> measured_gravity_body = detail::normalized_or(filtered_accel_, expected_gravity_body);
            correction = correction + expected_gravity_body.cross(measured_gravity_body) *
                                          (Config::kp_accel * last_accel_confidence_);
        }

        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            if (mag_valid && Config::enable_mag_correction)
            {
                const Vec<T, 3> expected_mag_body = orientation_.rotate_inverse(
                    detail::normalized_or(Config::world_magnetic_reference(), Vec<T, 3>(T(1), T(0), T(0))));
                const Vec<T, 3> measured_mag_body = detail::normalized_or(mag, expected_mag_body);
                correction =
                    correction + expected_mag_body.cross(measured_mag_body) * (Config::kp_mag * last_mag_confidence_);
                update_heading_state(true, dt);
            }
            else
            {
                update_heading_state(false, dt);
            }
        }
        else
        {
            update_heading_state(false, dt);
        }

        if (Config::estimate_gyro_bias && is_stationary(accel_valid ? accel : filtered_accel_, gyro))
        {
            gyro_bias_estimate_ = gyro_bias_estimate_ - correction * (Config::ki_gyro_bias * dt);
        }

        const Vec<T, 3> corrected_gyro = gyro - gyro_bias_estimate_ + correction;
        orientation_ = (orientation_ * detail::small_angle_quaternion(corrected_gyro * dt)).normalized();
        update_prediction_rate(corrected_gyro, true);
    }

    /// @brief Processes one update using the multiplicative EKF backend.
    void update_mekf(const Vec<T, 3>& accel, const Vec<T, 3>& gyro, const Vec<T, 3>& mag, T dt) noexcept
    {
        const bool accel_valid = accel_is_valid(accel);
        const bool gyro_valid = gyro_is_valid(gyro);
        const auto mag_measurement = evaluate_mag_measurement(mag);
        const bool mag_available = mag_measurement.available;
        const bool initialized_before_update = initialized_;

        initialize_from_measurement(accel, mag, mag_measurement.valid);
        if (!initialized_)
        {
            last_accel_confidence_ = T(0);
            last_mag_confidence_ = T(0);
            update_prediction_rate(Vec<T, 3>(), false);
            return;
        }

        if (accel_valid)
        {
            last_accel_confidence_ = detail::confidence_from_norm(accel.length(), Config::expected_gravity_norm,
                                                                  Config::accel_norm_min, Config::accel_norm_max);
        }
        else
        {
            set_flag(estimate_.flags, StatusFlag::accel_rejected);
            last_accel_confidence_ = T(0);
        }

        bool mag_valid = false;
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            mag_valid = mag_measurement.valid;
            if (mag_valid)
            {
                last_mag_confidence_ = mag_measurement.confidence;
                if (initialized_before_update)
                {
                    learn_mag_reference(mag_measurement.norm);
                }
            }
            else if (mag_measurement.available && Config::enable_mag_correction)
            {
                set_flag(estimate_.flags, StatusFlag::mag_rejected);
                last_mag_confidence_ = T(0);
            }
            else
            {
                last_mag_confidence_ = T(0);
            }
        }
        else
        {
            last_mag_confidence_ = T(0);
        }

        if (!(dt > T(0)) || !gyro_valid)
        {
            update_prediction_rate(gyro - gyro_bias_estimate_, gyro_valid);
            apply_measurement_only_update(accel, mag, accel_valid, mag_valid, dt);
            return;
        }

        const Vec<T, 3> omega = gyro - gyro_bias_estimate_;
        orientation_ = (orientation_ * detail::small_angle_quaternion(omega * dt)).normalized();

        Mat<T, 6, 6> f = Mat<T, 6, 6>::identity();
        const Mat<T, 3, 3> omega_hat = detail::skew(omega);
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t col = 0; col < 3; ++col)
            {
                f(row, col) -= omega_hat(row, col) * dt;
            }
            f(row, row + 3) = -dt;
        }

        covariance_ = f * covariance_ * f.transpose() +
                      detail::diagonal_process_covariance(Config::q_attitude, Config::q_gyro_bias, dt);

        if (accel_valid && Config::enable_accel_correction)
        {
            apply_vector_measurement_update(
                detail::normalized_or(accel, Config::world_gravity_direction()),
                detail::normalized_or(Config::world_gravity_direction(), Vec<T, 3>(T(0), T(0), T(-1))), Config::r_accel,
                Config::accel_nis_gate);
        }

        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            if (mag_valid && Config::enable_mag_correction)
            {
                apply_vector_measurement_update(
                    detail::normalized_or(mag, Config::world_magnetic_reference()),
                    detail::normalized_or(Config::world_magnetic_reference(), Vec<T, 3>(T(1), T(0), T(0))),
                    Config::r_mag, Config::mag_nis_gate);
                update_heading_state(true, dt);
            }
            else
            {
                update_heading_state(false, dt);
            }
        }
        else
        {
            update_heading_state(false, dt);
        }

        update_prediction_rate(omega, true);
    }

    /// @brief Applies one vector measurement update to the MEKF state and covariance.
    void apply_vector_measurement_update(const Vec<T, 3>& measured_body, const Vec<T, 3>& reference_world, T variance,
                                         T nis_gate) noexcept
    {
        const Vec<T, 3> predicted_body = orientation_.rotate_inverse(reference_world);
        const Vec<T, 3> residual = measured_body - predicted_body;

        Mat<T, 3, 6> h = Mat<T, 3, 6>::zero();
        const Mat<T, 3, 3> predicted_hat = detail::skew(predicted_body);
        for (std::size_t row = 0; row < 3; ++row)
        {
            for (std::size_t col = 0; col < 3; ++col)
            {
                h(row, col) = predicted_hat(row, col);
            }
        }

        const Mat<T, 3, 3> r = detail::diagonal_covariance(variance);
        const Mat<T, 3, 3> s = h * covariance_ * h.transpose() + r;
        Mat<T, 3, 3> s_inv = Mat<T, 3, 3>::zero();
        if (!s.inverse(s_inv))
        {
            return;
        }

        const Vec<T, 3> s_inv_residual = s_inv * residual;
        const T nis = residual.dot(s_inv_residual);
        if (nis > nis_gate)
        {
            return;
        }

        const Mat<T, 6, 3> k = covariance_ * h.transpose() * s_inv;
        const Vec<T, 6> delta = k * residual;
        const Vec<T, 3> delta_theta(delta[0], delta[1], delta[2]);
        const Vec<T, 3> delta_bias(delta[3], delta[4], delta[5]);
        orientation_ = (orientation_ * detail::small_angle_quaternion(delta_theta)).normalized();
        gyro_bias_estimate_ = gyro_bias_estimate_ + delta_bias;

        const Mat<T, 6, 6> i = Mat<T, 6, 6>::identity();
        const Mat<T, 6, 6> ikh = i - k * h;
        covariance_ = ikh * covariance_ * ikh.transpose() + k * r * k.transpose();
    }

    /// @brief Derives the current observability class from initialization and heading state.
    [[nodiscard]] auto current_observability() const noexcept -> Observability
    {
        if (!initialized_)
        {
            return Observability::none;
        }
        if constexpr (Config::sensor_model == SensorModel::accel_only)
        {
            return Observability::tilt_only;
        }
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            return heading_referenced_ ? Observability::full_3d : Observability::heading_with_drift;
        }
        return Observability::heading_with_drift;
    }

    /// @brief Computes entity confidence from measurement confidence and drift penalties.
    [[nodiscard]] auto current_confidence(T accel_confidence, T mag_confidence) const noexcept -> T
    {
        if (!initialized_)
        {
            return T(0);
        }

        T confidence = static_cast<T>(0.95);
        if constexpr (Config::sensor_model == SensorModel::accel_only)
        {
            confidence *= accel_confidence;
        }
        else
        {
            confidence *= std::max(accel_confidence, static_cast<T>(0.5));
            if constexpr (Config::sensor_model == SensorModel::imu9)
            {
                confidence *=
                    heading_referenced_ ? std::max(mag_confidence, static_cast<T>(0.5)) : static_cast<T>(0.75);
            }
            else
            {
                confidence *= static_cast<T>(0.8);
            }
        }

        if (estimated_drift_rad_ > T(0))
        {
            confidence *= detail::saturate(T(1) - (estimated_drift_rad_ / Config::drift_confidence_limit_rad));
        }

        return detail::saturate(confidence);
    }
};

template<typename T, typename LeftConfig, typename RightConfig, typename RelativeConfig = DefaultRelativeAngleConfig<T>>
class RelativeAngleEstimator
{
public:
    using LeftEstimator = OrientationEstimator<T, LeftConfig>;
    using RightEstimator = OrientationEstimator<T, RightConfig>;
    using LeftSample = typename LeftEstimator::Sample;
    using RightSample = typename RightEstimator::Sample;

    /// @brief Creates a relative estimator with default-initialized left and right entities.
    RelativeAngleEstimator() noexcept = default;

    static_assert(std::is_floating_point<T>::value, "RelativeAngleEstimator requires a floating-point scalar type.");
    static_assert(
        RelativeConfig::allow_drift_with_quality_flags ||
            !detail::output_requires_heading(RelativeConfig::default_output) ||
            (detail::has_absolute_heading_capability<LeftConfig>() &&
             detail::has_absolute_heading_capability<RightConfig>()),
        "The selected default output requires heading observability, but the policy forbids drift-enabled outputs.");

    /// @brief Updates the left entity estimator.
    void update_left(const LeftSample& sample) noexcept { left_.update(sample); }

    /// @brief Updates the right entity estimator.
    void update_right(const RightSample& sample) noexcept { right_.update(sample); }

    /// @brief Resets both entity estimators and clears any captured reference pose.
    void reset() noexcept
    {
        left_.reset();
        right_.reset();
        reference_pose_ = Quaternion<T>::identity();
        reference_valid_ = false;
    }

    /// @brief Sets the left-entity runtime calibration.
    void set_left_calibration(const SensorCalibration<T>& calibration) noexcept { left_.set_calibration(calibration); }

    /// @brief Sets the right-entity runtime calibration.
    void set_right_calibration(const SensorCalibration<T>& calibration) noexcept
    {
        right_.set_calibration(calibration);
    }

    /// @brief Captures the current relative pose as the neutral reference.
    auto capture_reference_pose() noexcept -> bool
    {
        const OrientationEstimate<T>& left_estimate = left_.estimate();
        const OrientationEstimate<T>& right_estimate = right_.estimate();
        if (!left_estimate.valid || !right_estimate.valid)
        {
            return false;
        }

        reference_pose_ = (left_estimate.orientation.inverse_unit() * right_estimate.orientation).normalized();
        reference_valid_ = true;
        return true;
    }

    /// @brief Clears the captured neutral reference pose.
    void clear_reference_pose() noexcept
    {
        reference_pose_ = Quaternion<T>::identity();
        reference_valid_ = false;
    }

    /// @brief Computes the relative result using the compile-time default scalar mode.
    [[nodiscard]] auto compute() const noexcept -> RelativeAngleResult<T>
    {
        return compute_impl(RelativeConfig::default_output);
    }

    /// @brief Computes the relative result using a caller-selected scalar mode.
    [[nodiscard]] auto compute_scalar(PrimaryScalarOutput mode) const noexcept -> RelativeAngleResult<T>
    {
        static_assert(RelativeConfig::allow_runtime_output_override,
                      "This relative-angle policy disables runtime output-mode overrides.");
        return compute_impl(mode);
    }

    /// @brief Returns the left entity estimator.
    [[nodiscard]] auto left_estimator() const noexcept -> const LeftEstimator& { return left_; }

    /// @brief Returns the right entity estimator.
    [[nodiscard]] auto right_estimator() const noexcept -> const RightEstimator& { return right_; }

private:
    LeftEstimator left_{};
    RightEstimator right_{};
    Quaternion<T> reference_pose_ = Quaternion<T>::identity();
    bool reference_valid_{};
    static constexpr T frame_alignment_min_cosine_ = static_cast<T>(0.9998476951563913);
    static constexpr T hinge_axis_alignment_min_cosine_ = static_cast<T>(0.7071067811865476);

    [[nodiscard]] static auto scalar_is_finite(T value) noexcept -> bool { return std::isfinite(value); }

    [[nodiscard]] static auto axis_is_valid(const Vec<T, 3>& axis) noexcept -> bool
    {
        return detail::is_usable_direction(axis);
    }

    [[nodiscard]] static auto uses_hinge_axis(PrimaryScalarOutput mode) noexcept -> bool
    {
        return mode == PrimaryScalarOutput::hinge_twist || mode == PrimaryScalarOutput::swing_angle;
    }

    [[nodiscard]] static auto policy_is_valid(PrimaryScalarOutput mode) noexcept -> bool
    {
        if (!scalar_is_finite(RelativeConfig::max_pair_skew_s) ||
            !scalar_is_finite(RelativeConfig::max_alignment_horizon_s) ||
            !scalar_is_finite(RelativeConfig::min_confidence_to_publish) ||
            !scalar_is_finite(RelativeConfig::nominal_drift_limit_rad))
        {
            return false;
        }

        if (RelativeConfig::max_pair_skew_s < T(0) || RelativeConfig::max_alignment_horizon_s < T(0) ||
            RelativeConfig::min_confidence_to_publish < T(0) || RelativeConfig::min_confidence_to_publish > T(1) ||
            !(RelativeConfig::nominal_drift_limit_rad > T(0)))
        {
            return false;
        }

        if (!axis_is_valid(LeftConfig::body_down_axis()) || !axis_is_valid(RightConfig::body_down_axis()))
        {
            return false;
        }

        if (detail::output_requires_heading(mode) &&
            (!axis_is_valid(LeftConfig::body_heading_axis()) || !axis_is_valid(RightConfig::body_heading_axis())))
        {
            return false;
        }

        return !uses_hinge_axis(mode) ||
               (axis_is_valid(RelativeConfig::hinge_axis_left()) && axis_is_valid(RelativeConfig::hinge_axis_right()));
    }

    [[nodiscard]] static auto frames_are_consistent(PrimaryScalarOutput mode) noexcept -> bool
    {
        const T gravity_alignment = detail::direction_cosine(LeftConfig::world_gravity_direction(),
                                                             RightConfig::world_gravity_direction(), T(-1));
        if (gravity_alignment < frame_alignment_min_cosine_)
        {
            return false;
        }

        if (detail::output_requires_heading(mode) && detail::has_absolute_heading_capability<LeftConfig>() &&
            detail::has_absolute_heading_capability<RightConfig>())
        {
            const T magnetic_alignment = detail::direction_cosine(LeftConfig::world_magnetic_reference(),
                                                                  RightConfig::world_magnetic_reference(), T(-1));
            if (magnetic_alignment < frame_alignment_min_cosine_)
            {
                return false;
            }
        }

        return true;
    }

    static void reject_invalid_configuration(RelativeAngleResult<T>& result, bool heading_related) noexcept
    {
        set_flag(result.primary.flags, StatusFlag::configuration_invalid);
        set_flag(result.primary.flags, StatusFlag::output_not_supported);
        if (heading_related)
        {
            set_flag(result.primary.flags, StatusFlag::heading_unobservable);
        }
        result.primary.observability = Observability::none;
        result.primary.quality = SolutionQuality::invalid;
        result.primary.valid = false;
        result.valid = false;
    }

    [[nodiscard]] static auto resolve_hinge_axis_left(const Quaternion<T>& q_left_to_right,
                                                      Vec<T, 3>& axis_left) noexcept -> bool
    {
        if (!axis_is_valid(RelativeConfig::hinge_axis_left()) || !axis_is_valid(RelativeConfig::hinge_axis_right()))
        {
            return false;
        }

        const Vec<T, 3> left_axis =
            detail::normalized_or(RelativeConfig::hinge_axis_left(), Vec<T, 3>(T(0), T(0), T(1)));
        Vec<T, 3> right_axis_left =
            q_left_to_right.inverse_unit().rotate(detail::normalized_or(RelativeConfig::hinge_axis_right(), left_axis));
        if (!axis_is_valid(right_axis_left))
        {
            return false;
        }

        right_axis_left = detail::normalized_or(right_axis_left, left_axis);
        const T axis_alignment = std::abs(left_axis.dot(right_axis_left));
        if (axis_alignment < hinge_axis_alignment_min_cosine_)
        {
            return false;
        }

        if (left_axis.dot(right_axis_left) < T(0))
        {
            right_axis_left = -right_axis_left;
        }

        axis_left = detail::normalized_or(left_axis + right_axis_left, left_axis);
        return true;
    }

    /// @brief Computes the full relative result for a selected scalar output mode.
    [[nodiscard]] auto compute_impl(PrimaryScalarOutput mode) const noexcept -> RelativeAngleResult<T>
    {
        RelativeAngleResult<T> result;
        result.left_entity = left_.estimate();
        result.right_entity = right_.estimate();
        result.primary.mode = mode;
        result.reference_active = reference_valid_ && RelativeConfig::apply_reference_pose;

        if (!result.left_entity.valid || !result.right_entity.valid)
        {
            result.primary.flags = result.left_entity.flags | result.right_entity.flags;
            result.primary.quality = SolutionQuality::invalid;
            result.primary.observability = Observability::none;
            return result;
        }

        if (!policy_is_valid(mode) || !frames_are_consistent(mode))
        {
            reject_invalid_configuration(result, detail::output_requires_heading(mode));
            return result;
        }

        const T latest_time = std::max(result.left_entity.timestamp_s, result.right_entity.timestamp_s);
        const T left_age = latest_time - result.left_entity.timestamp_s;
        const T right_age = latest_time - result.right_entity.timestamp_s;
        result.sample_skew_s = std::abs(result.left_entity.timestamp_s - result.right_entity.timestamp_s);

        if (left_age > LeftConfig::max_sample_age_s)
        {
            set_flag(result.primary.flags, StatusFlag::left_sample_stale);
        }
        if (right_age > RightConfig::max_sample_age_s)
        {
            set_flag(result.primary.flags, StatusFlag::right_sample_stale);
        }
        if (result.sample_skew_s > RelativeConfig::max_pair_skew_s)
        {
            set_flag(result.primary.flags, StatusFlag::pair_time_skew_exceeded);
        }

        const Quaternion<T> left_orientation =
            left_.predict_orientation(latest_time, RelativeConfig::max_alignment_horizon_s);
        const Quaternion<T> right_orientation =
            right_.predict_orientation(latest_time, RelativeConfig::max_alignment_horizon_s);

        Quaternion<T> q_left_to_right = (left_orientation.inverse_unit() * right_orientation).normalized();
        if (reference_valid_ && RelativeConfig::apply_reference_pose)
        {
            q_left_to_right = (reference_pose_.inverse_unit() * q_left_to_right).normalized();
        }
        result.q_left_to_right = q_left_to_right;
        result.relative_euler_rad = q_left_to_right.to_euler();

        const Vec<T, 3> left_down_world = left_orientation.rotate(LeftConfig::body_down_axis());
        const Vec<T, 3> right_down_world = right_orientation.rotate(RightConfig::body_down_axis());
        const Vec<T, 3> world_up =
            detail::normalized_or(-LeftConfig::world_gravity_direction(), Vec<T, 3>(T(0), T(0), T(1)));

        const Vec<T, 3> left_heading_world =
            detail::project_onto_plane(left_orientation.rotate(LeftConfig::body_heading_axis()), world_up);
        const Vec<T, 3> right_heading_world =
            detail::project_onto_plane(right_orientation.rotate(RightConfig::body_heading_axis()), world_up);
        Vec<T, 3> hinge_axis_left =
            detail::normalized_or(RelativeConfig::hinge_axis_left(), Vec<T, 3>(T(0), T(0), T(1)));
        const bool hinge_axes_consistent = resolve_hinge_axis_left(q_left_to_right, hinge_axis_left);

        result.shortest_3d_angle_rad = detail::shortest_angle_from_quaternion(q_left_to_right);
        result.tilt_angle_rad = detail::normalized_or(left_down_world, LeftConfig::body_down_axis())
                                    .angle(detail::normalized_or(right_down_world, RightConfig::body_down_axis()));
        result.heading_delta_rad =
            (left_heading_world.length_squared() > std::numeric_limits<T>::epsilon() &&
             right_heading_world.length_squared() > std::numeric_limits<T>::epsilon())
                ? detail::normalized_or(left_heading_world, Vec<T, 3>(T(1), T(0), T(0)))
                      .signed_angle(detail::normalized_or(right_heading_world, Vec<T, 3>(T(1), T(0), T(0))), world_up)
                : T(0);
        result.hinge_twist_rad = detail::twist_angle_about_axis(q_left_to_right, hinge_axis_left);
        result.swing_angle_rad = detail::swing_angle_about_axis(q_left_to_right, hinge_axis_left);

        if (detail::output_requires_heading(mode) &&
            (!detail::is_usable_direction(left_heading_world) || !detail::is_usable_direction(right_heading_world)))
        {
            reject_invalid_configuration(result, true);
            return result;
        }

        if (uses_hinge_axis(mode) && !hinge_axes_consistent)
        {
            reject_invalid_configuration(result, false);
            return result;
        }

        const Observability pair_observability =
            detail::combined_observability(result.left_entity.observability, result.right_entity.observability);
        result.primary.observability = effective_output_observability(mode, pair_observability);

        if (result.primary.observability == Observability::none)
        {
            set_flag(result.primary.flags, StatusFlag::heading_unobservable);
            set_flag(result.primary.flags, StatusFlag::output_not_supported);
            result.primary.quality = SolutionQuality::invalid;
            result.primary.valid = false;
            result.valid = false;
            return result;
        }

        result.primary.estimated_drift_rad =
            std::sqrt(result.left_entity.estimated_drift_rad * result.left_entity.estimated_drift_rad +
                      result.right_entity.estimated_drift_rad * result.right_entity.estimated_drift_rad);

        if (result.primary.estimated_drift_rad > RelativeConfig::nominal_drift_limit_rad)
        {
            set_flag(result.primary.flags, StatusFlag::drift_exceeds_nominal);
        }

        result.primary.angle_rad = extract_scalar(mode, result);

        T confidence = result.left_entity.confidence * result.right_entity.confidence;
        if (result.sample_skew_s > RelativeConfig::max_pair_skew_s)
        {
            confidence *= static_cast<T>(0.5);
        }
        if (result.primary.observability == Observability::heading_with_drift)
        {
            confidence *= static_cast<T>(0.8);
        }
        confidence *=
            detail::saturate(T(1) - (result.primary.estimated_drift_rad / RelativeConfig::nominal_drift_limit_rad));
        result.primary.confidence = detail::saturate(confidence);
        result.primary.quality = detail::quality_from_confidence(result.primary.confidence, result.primary.flags);
        result.primary.valid = result.primary.confidence >= RelativeConfig::min_confidence_to_publish &&
                               result.primary.quality != SolutionQuality::invalid;
        result.valid = result.primary.valid;
        return result;
    }

    /// @brief Determines the observability required or allowed for the selected scalar output.
    [[nodiscard]] static auto effective_output_observability(PrimaryScalarOutput mode,
                                                             Observability pair_observability) noexcept -> Observability
    {
        if (!detail::output_requires_heading(mode))
        {
            return detail::observability_rank(pair_observability) >=
                           detail::observability_rank(Observability::tilt_only)
                       ? Observability::tilt_only
                       : Observability::none;
        }

        if (pair_observability == Observability::full_3d)
        {
            return Observability::full_3d;
        }
        if (pair_observability == Observability::heading_with_drift && RelativeConfig::allow_drift_with_quality_flags)
        {
            return Observability::heading_with_drift;
        }
        return Observability::none;
    }

    /// @brief Extracts the selected scalar output from a populated relative-angle result.
    [[nodiscard]] static auto extract_scalar(PrimaryScalarOutput mode,
                                             const RelativeAngleResult<T>& result) noexcept -> T
    {
        switch (mode)
        {
        case PrimaryScalarOutput::shortest_3d_angle:
            return result.shortest_3d_angle_rad;
        case PrimaryScalarOutput::tilt_angle:
            return result.tilt_angle_rad;
        case PrimaryScalarOutput::heading_delta:
            return result.heading_delta_rad;
        case PrimaryScalarOutput::hinge_twist:
            return result.hinge_twist_rad;
        case PrimaryScalarOutput::swing_angle:
            return result.swing_angle_rad;
        case PrimaryScalarOutput::roll_delta:
            return result.relative_euler_rad[Quaternion<T>::ROLL];
        case PrimaryScalarOutput::pitch_delta:
            return result.relative_euler_rad[Quaternion<T>::PITCH];
        case PrimaryScalarOutput::yaw_delta:
            return result.relative_euler_rad[Quaternion<T>::YAW];
        }
        return T(0);
    }
};

}  // namespace fusion
}  // namespace microla
