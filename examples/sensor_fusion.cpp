// SPDX-License-Identifier: Apache-2.0
/// @file sensor_fusion.cpp
/// @brief Relative sensor-fusion example using microla::fusion
/// @details Demonstrates a mixed 9-axis plus 6-axis setup, compile-time policies, runtime
///          scalar-output selection, and drift-aware diagnostics.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/constants.hpp>
#include <microla/sensor_fusion.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

using microla::Quaternion;
using microla::Vec;
namespace constants = microla::constants;
namespace fusion = microla::fusion;

using Vec3f = Vec<float, 3>;

namespace
{

auto deg_to_rad(float degrees) noexcept -> float
{
    return degrees * constants::deg_to_rad<float>();
}

auto rad_to_deg(float radians) noexcept -> float
{
    return radians * constants::rad_to_deg<float>();
}

auto make_accel_body(const Quaternion<float>& body_to_world,
                     float gravity = constants::gravity<float>()) noexcept -> Vec3f
{
    const Vec3f world_gravity(0.0F, 0.0F, -gravity);
    return body_to_world.inverse().rotate(world_gravity);
}

auto make_mag_body(const Quaternion<float>& body_to_world, float field_strength = 50.0F) noexcept -> Vec3f
{
    const Vec3f world_mag(field_strength, 0.0F, 0.0F);
    return body_to_world.inverse().rotate(world_mag);
}

struct AnchorImu9Config : fusion::DefaultImu9MahonyConfig<float>
{
    static constexpr float gravity_filter_alpha = 0.35F;
    static constexpr float kp_mag = 1.0F;
};

struct FollowerImu6Config : fusion::DefaultImu6MahonyConfig<float>
{
    static constexpr float gravity_filter_alpha = 0.35F;
    static constexpr float drift_rate_without_heading_rad_per_s = 0.03F;
};

struct JointAngleConfig : fusion::DefaultRelativeAngleConfig<float>
{
    static constexpr fusion::PrimaryScalarOutput default_output = fusion::PrimaryScalarOutput::hinge_twist;
    static constexpr bool apply_reference_pose = true;
    static constexpr float min_confidence_to_publish = 0.30F;

    static auto hinge_axis_left() noexcept -> Vec3f { return {0.0F, 0.0F, 1.0F}; }
    static auto hinge_axis_right() noexcept -> Vec3f { return {0.0F, 0.0F, 1.0F}; }
};

using ExampleEstimator = fusion::RelativeAngleEstimator<float, AnchorImu9Config, FollowerImu6Config, JointAngleConfig>;

auto observability_name(fusion::Observability observability) noexcept -> const char*
{
    switch (observability)
    {
    case fusion::Observability::none:
        return "none";
    case fusion::Observability::tilt_only:
        return "tilt_only";
    case fusion::Observability::heading_with_drift:
        return "heading_with_drift";
    case fusion::Observability::full_3d:
        return "full_3d";
    }
    return "unknown";
}

auto quality_name(fusion::SolutionQuality quality) noexcept -> const char*
{
    switch (quality)
    {
    case fusion::SolutionQuality::invalid:
        return "invalid";
    case fusion::SolutionQuality::degraded:
        return "degraded";
    case fusion::SolutionQuality::usable:
        return "usable";
    case fusion::SolutionQuality::nominal:
        return "nominal";
    }
    return "unknown";
}

void print_flags(std::ostream& out, fusion::StatusFlags flags)
{
    bool printed = false;
    const auto emit = [&](fusion::StatusFlag flag, const char* label)
    {
        if (!fusion::has_flag(flags, flag))
        {
            return;
        }
        if (printed)
        {
            out << ',';
        }
        out << label;
        printed = true;
    };

    emit(fusion::StatusFlag::sample_time_invalid, "sample_time_invalid");
    emit(fusion::StatusFlag::gyro_rejected, "gyro_rejected");
    emit(fusion::StatusFlag::accel_rejected, "accel_rejected");
    emit(fusion::StatusFlag::mag_rejected, "mag_rejected");
    emit(fusion::StatusFlag::propagation_only, "propagation_only");
    emit(fusion::StatusFlag::startup_not_initialized, "startup_not_initialized");
    emit(fusion::StatusFlag::left_sample_stale, "left_sample_stale");
    emit(fusion::StatusFlag::right_sample_stale, "right_sample_stale");
    emit(fusion::StatusFlag::pair_time_skew_exceeded, "pair_time_skew_exceeded");
    emit(fusion::StatusFlag::heading_unobservable, "heading_unobservable");
    emit(fusion::StatusFlag::drift_exceeds_nominal, "drift_exceeds_nominal");
    emit(fusion::StatusFlag::output_not_supported, "output_not_supported");
    emit(fusion::StatusFlag::calibration_invalid, "calibration_invalid");
    emit(fusion::StatusFlag::configuration_invalid, "configuration_invalid");

    if (!printed)
    {
        out << "none";
    }
}

auto make_anchor_sample(float timestamp_s, const Quaternion<float>& orientation) noexcept -> fusion::Imu9Sample<float>
{
    return fusion::Imu9Sample<float>{timestamp_s, Vec3f(0.0F, 0.0F, 0.0F), make_accel_body(orientation),
                                     make_mag_body(orientation)};
}

auto make_follower_sample(float timestamp_s, const Quaternion<float>& orientation,
                          float yaw_rate_rad_s) noexcept -> fusion::Imu6Sample<float>
{
    return fusion::Imu6Sample<float>{timestamp_s, Vec3f(0.0F, 0.0F, yaw_rate_rad_s), make_accel_body(orientation)};
}

}  // namespace

auto main() -> int
{
    ExampleEstimator estimator;

    const Quaternion<float> anchor_orientation = Quaternion<float>::identity();
    Quaternion<float> follower_orientation = Quaternion<float>::identity();
    constexpr float dt_s = 0.1F;
    const float yaw_rate_rad_s = deg_to_rad(30.0F);

    estimator.update_left(make_anchor_sample(0.0F, anchor_orientation));
    estimator.update_right(make_follower_sample(0.0F, follower_orientation, 0.0F));

    if (!estimator.capture_reference_pose())
    {
        std::cerr << "Failed to capture neutral reference pose\n";
        return 1;
    }

    std::cout << "MicroLA - Relative Sensor Fusion Example\n";
    std::cout << "========================================\n\n";
    std::cout << "Scenario: a 9-axis anchor is paired with a 6-axis follower.\n";
    std::cout << "The primary output is hinge twist, while runtime override is used to inspect tilt.\n\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time  Twist(deg)  Tilt(deg)  Drift(deg)  Conf  Obs                   Quality   Follower Flags\n";
    std::cout << "------------------------------------------------------------------------------------------------\n";

    fusion::RelativeAngleResult<float> latest_result;
    fusion::RelativeAngleResult<float> latest_tilt_result;

    for (int step = 1; step <= 10; ++step)
    {
        const float timestamp_s = static_cast<float>(step) * dt_s;
        follower_orientation =
            Quaternion<float>::from_axis_angle(Vec3f(0.0F, 0.0F, 1.0F), yaw_rate_rad_s * timestamp_s);

        estimator.update_left(make_anchor_sample(timestamp_s, anchor_orientation));
        estimator.update_right(make_follower_sample(timestamp_s, follower_orientation, yaw_rate_rad_s));

        latest_result = estimator.compute();
        latest_tilt_result = estimator.compute_scalar(fusion::PrimaryScalarOutput::tilt_angle);

        std::cout << std::setw(4) << timestamp_s << "  " << std::setw(10) << rad_to_deg(latest_result.primary.angle_rad)
                  << "  " << std::setw(9) << rad_to_deg(latest_tilt_result.primary.angle_rad) << "  " << std::setw(10)
                  << rad_to_deg(latest_result.primary.estimated_drift_rad) << "  " << std::setw(4)
                  << latest_result.primary.confidence << "  " << std::setw(20)
                  << observability_name(latest_result.primary.observability) << "  " << std::setw(8)
                  << quality_name(latest_result.primary.quality) << "   ";
        print_flags(std::cout, latest_result.right_entity.flags);
        std::cout << "\n";
    }

    const fusion::RelativeAngleResult<float> heading_result =
        estimator.compute_scalar(fusion::PrimaryScalarOutput::heading_delta);
    const float final_twist_deg = rad_to_deg(latest_result.primary.angle_rad);

    std::cout << "\nFinal hinge twist: " << final_twist_deg << " deg\n";
    std::cout << "Final heading delta: " << rad_to_deg(heading_result.primary.angle_rad) << " deg\n";
    std::cout << "Final observability: " << observability_name(latest_result.primary.observability) << '\n';

    if (!latest_result.valid || !heading_result.primary.valid)
    {
        std::cerr << "Sensor-fusion example did not produce a valid final result\n";
        return 1;
    }

    if (std::abs(final_twist_deg - 30.0F) > 2.0F)
    {
        std::cerr << "Unexpected final hinge angle: " << final_twist_deg << " deg\n";
        return 1;
    }

    if (latest_result.primary.observability != fusion::Observability::heading_with_drift)
    {
        std::cerr << "Expected drift-aware heading observability from the mixed 9-axis/6-axis pair\n";
        return 1;
    }

    if (!fusion::has_flag(latest_result.right_entity.flags, fusion::StatusFlag::propagation_only))
    {
        std::cerr << "Expected the 6-axis follower to report propagation-only heading updates\n";
        return 1;
    }

    return 0;
}
