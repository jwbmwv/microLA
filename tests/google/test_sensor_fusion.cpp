// SPDX-License-Identifier: Apache-2.0
/// @file test_sensor_fusion.cpp
/// @brief Unit tests for relative sensor-fusion estimators

#include <gtest/gtest.h>

#include <microla/sensor_fusion.hpp>

#include <cmath>
#include <limits>

using namespace microla;
using namespace microla::fusion;

namespace
{

template<typename T>
auto deg_to_rad_local(T degrees) noexcept -> T
{
    return degrees * constants::deg_to_rad<T>();
}

template<typename T>
auto make_accel_body(const Quaternion<T>& body_to_world, T gravity = constants::gravity<T>()) noexcept -> Vec<T, 3>
{
    const Vec<T, 3> world_gravity(0, 0, -gravity);
    return body_to_world.inverse().rotate(world_gravity);
}

template<typename T>
auto make_mag_body(const Quaternion<T>& body_to_world, T field_strength = static_cast<T>(50.0)) noexcept -> Vec<T, 3>
{
    const Vec<T, 3> world_mag(field_strength, 0, 0);
    return body_to_world.inverse().rotate(world_mag);
}

template<typename T>
auto quaternion_distance(const Quaternion<T>& lhs, const Quaternion<T>& rhs) noexcept -> T
{
    const Quaternion<T> delta = (lhs.inverse_unit() * rhs).normalized();
    return T(2) * std::acos(std::min(T(1), std::abs(delta.w())));
}

template<typename T>
struct LeftAccelConfig : DefaultAccelOnlyConfig<T>
{
};

template<typename T>
struct RightAccelConfig : DefaultAccelOnlyConfig<T>
{
};

template<typename T>
struct TiltRelativeConfig : DefaultRelativeAngleConfig<T>
{
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::tilt_angle;
};

template<typename T>
struct HeadingRelativeConfig : DefaultRelativeAngleConfig<T>
{
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::heading_delta;
};

template<typename T>
struct SkewTolerantHeadingRelativeConfig : HeadingRelativeConfig<T>
{
    static constexpr T min_confidence_to_publish = static_cast<T>(0.2);
};

template<typename T>
struct ReferencedHeadingRelativeConfig : HeadingRelativeConfig<T>
{
    static constexpr bool apply_reference_pose = true;
};

template<typename T>
struct ReferencedTiltRelativeConfig : TiltRelativeConfig<T>
{
    static constexpr bool apply_reference_pose = true;
};

template<typename T>
struct MismatchedWorldImu9Config : DefaultImu9MahonyConfig<T>
{
    static auto world_gravity_direction() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(1)); }
};

template<typename T>
struct InconsistentHingeRelativeConfig : HeadingRelativeConfig<T>
{
    static auto hinge_axis_left() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(0), T(0), T(1)); }
    static auto hinge_axis_right() noexcept -> Vec<T, 3> { return Vec<T, 3>(T(1), T(0), T(0)); }
};

template<typename T>
struct LeftImu6Config : DefaultImu6MahonyConfig<T>
{
};

template<typename T>
struct RightImu6Config : DefaultImu6MahonyConfig<T>
{
};

template<typename T>
struct LeftImu9EkfConfig : DefaultImu9EkfConfig<T>
{
};

template<typename T>
struct RightImu9EkfConfig : DefaultImu9EkfConfig<T>
{
};

template<typename T>
struct StrictNisImu9EkfConfig : DefaultImu9EkfConfig<T>
{
    static constexpr T accel_nis_gate = static_cast<T>(1e-12);
    static constexpr T mag_nis_gate = static_cast<T>(1e-12);
};

template<typename T>
struct NoDriftHeadingRelativeConfig : DefaultRelativeAngleConfig<T>
{
    static constexpr bool allow_drift_with_quality_flags = false;
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::tilt_angle;
};

}  // namespace

TEST(SensorFusionOrientationTest, DisabledMekfBiasEstimationPreservesBiasAndCrossCovariance)
{
    struct Config : DefaultImu6EkfConfig<double>
    {
        enum
        {
            estimate_gyro_bias = false
        };
    };
    OrientationEstimator<double, Config> estimator;
    const Vec<double, 3> gravity(0, 0, -constants::gravity<double>());
    const auto tilted = make_accel_body(Quaternion<double>::from_euler(0.2, -0.1, 0.0));
    estimator.update(Imu6Sample<double>{0, {}, gravity});
    for (int step = 1; step <= 300; ++step)
    {
        estimator.update(Imu6Sample<double>{step * 0.01, {}, tilted});
    }
    EXPECT_TRUE(estimator.gyro_bias_estimate().is_zero());
    for (std::size_t row = 0; row < 3; ++row)
    {
        EXPECT_DOUBLE_EQ(estimator.error_covariance()(row + 3, row + 3), 0.1);
        for (std::size_t col = 3; col < 6; ++col)
        {
            EXPECT_DOUBLE_EQ(estimator.error_covariance()(row, col), 0.0);
        }
    }
}

TEST(SensorFusionOrientationTest, DisabledMahonyAccelCorrectionHoldsAttitudeDuringGyroDropout)
{
    struct Config : DefaultImu6MahonyConfig<double>
    {
        enum
        {
            enable_accel_correction = false,
            estimate_gyro_bias = false
        };
    };
    OrientationEstimator<double, Config> estimator;
    estimator.update(Imu6Sample<double>{0, {}, make_accel_body(Quaternion<double>::identity())});
    const auto before = estimator.estimate().orientation;
    estimator.update(Imu6Sample<double>{0.1, Vec<double, 3>(std::numeric_limits<double>::quiet_NaN(), 0, 0),
                                        make_accel_body(Quaternion<double>::from_euler(0.2, -0.1, 0.0))});
    EXPECT_TRUE(estimator.estimate().orientation.approx_equal(before, 1e-12));
    EXPECT_FALSE(estimator.estimate().valid);
    EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::orientation_held));
}

TEST(SensorFusionTest, StationaryImu6TiltRemainsValidAfterHeadingConfidenceExpires)
{
    RelativeAngleEstimator<double, DefaultImu6MahonyConfig<double>, DefaultImu6MahonyConfig<double>> estimator;
    for (int step = 0; step <= 600; ++step)
    {
        const Imu6Sample<double> sample{step * 0.1, {}, make_accel_body(Quaternion<double>::identity())};
        estimator.update_left(sample);
        estimator.update_right(sample);
        const auto tilt = estimator.compute_scalar(PrimaryScalarOutput::tilt_angle);
        ASSERT_TRUE(tilt.valid) << step;
        EXPECT_DOUBLE_EQ(tilt.primary.estimated_drift_rad, 0.0);
        EXPECT_NEAR(tilt.primary.angle_rad, 0.0, 1e-12);
    }
    EXPECT_FALSE(estimator.compute_scalar(PrimaryScalarOutput::heading_delta).valid);
}

TEST(SensorFusionTest, ReferenceCaptureUsesTheSameTimeAlignmentAsCompute)
{
    RelativeAngleEstimator<double, DefaultImu9MahonyConfig<double>, DefaultImu9MahonyConfig<double>,
                           ReferencedHeadingRelativeConfig<double>>
        estimator;
    const Vec<double, 3> rate(0, 0, 1);
    const auto identity = Quaternion<double>::identity();
    estimator.update_left(Imu9Sample<double>{0, rate, make_accel_body(identity), make_mag_body(identity)});
    estimator.update_right(Imu9Sample<double>{0, rate, make_accel_body(identity), make_mag_body(identity)});
    const auto rotated = Quaternion<double>::from_euler(0, 0, 0.01);
    estimator.update_right(Imu9Sample<double>{0.01, rate, make_accel_body(rotated), make_mag_body(rotated)});
    ASSERT_TRUE(estimator.capture_reference_pose());
    const auto result = estimator.compute();
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.primary.angle_rad, 0.0, 1e-12);
    ASSERT_TRUE(estimator.capture_reference_pose());
    EXPECT_NEAR(estimator.compute().primary.angle_rad, 0.0, 1e-12);
}

TEST(SensorFusionOrientationTest, AccelOnlyRejectsInvalidStartupSample)
{
    OrientationEstimator<float, DefaultAccelOnlyConfig<float>> estimator;

    estimator.update(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(estimate.observability, Observability::none);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::accel_rejected));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::startup_not_initialized));
}

TEST(SensorFusionOrientationTest, RejectsNonFiniteStartupTimestamp)
{
    OrientationEstimator<float, DefaultAccelOnlyConfig<float>> estimator;

    estimator.update(AccelSample<float>{std::numeric_limits<float>::quiet_NaN(),
                                        Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(estimate.observability, Observability::none);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::sample_time_invalid));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::startup_not_initialized));
    EXPECT_FLOAT_EQ(estimate.timestamp_s, 0.0F);
}

TEST(SensorFusionOrientationTest, OutOfOrderSampleDoesNotChangeOrientation)
{
    OrientationEstimator<float, DefaultAccelOnlyConfig<float>> estimator;
    const Vec<float, 3> level_accel(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> tilted_accel(0.0F, -constants::gravity<float>(), 0.0F);

    estimator.update(AccelSample<float>{1.0F, level_accel});
    ASSERT_TRUE(estimator.estimate().valid);
    const Quaternion<float> orientation_before = estimator.orientation();

    estimator.update(AccelSample<float>{0.5F, tilted_accel});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_FALSE(estimate.valid);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::sample_time_invalid));
    EXPECT_NEAR(quaternion_distance(estimator.orientation(), orientation_before), 0.0F, 1e-6F);
    EXPECT_FLOAT_EQ(estimate.timestamp_s, 1.0F);
}

template<typename Config>
void expect_stale_samples_preserve_estimator_state()
{
    const Vec<double, 3> gravity(0.0, 0.0, -constants::gravity<double>());
    const Vec<double, 3> magnetic_field(0.5, 0.0, 0.0);
    OrientationEstimator<double, Config> reference;
    reference.update(Imu9Sample<double>{0.0, Vec<double, 3>(), gravity, magnetic_field});
    reference.update(Imu9Sample<double>{0.1, Vec<double, 3>(0.0, 0.0, 0.5), gravity, magnetic_field});
    const Quaternion<double> tilted = Quaternion<double>::from_euler(0.3, -0.2, 0.4);

    for (double timestamp : {0.1, 0.1 + Config::min_dt_s * 0.5, 0.05})
    {
        auto estimator = reference;
        auto control = reference;
        estimator.update(
            Imu9Sample<double>{timestamp, Vec<double, 3>(), make_accel_body(tilted), make_mag_body(tilted, 0.5)});
        EXPECT_FALSE(estimator.estimate().valid);
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::sample_time_invalid));
        EXPECT_FALSE(has_flag(estimator.estimate().flags, StatusFlag::gyro_rejected));
        EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 0.1);
        EXPECT_LT(quaternion_distance(estimator.orientation(), control.orientation()), 1e-6);
        EXPECT_EQ(estimator.gyro_bias_estimate(), control.gyro_bias_estimate());

        const Imu9Sample<double> next_sample{0.11, Vec<double, 3>(), gravity, magnetic_field};
        estimator.update(next_sample);
        control.update(next_sample);
        EXPECT_LT(quaternion_distance(estimator.orientation(), control.orientation()), 1e-6);
        EXPECT_EQ(estimator.gyro_bias_estimate(), control.gyro_bias_estimate());
    }
}

TEST(SensorFusionOrientationTest, MahonyRejectsStaleSamplesWithoutStateChanges)
{
    expect_stale_samples_preserve_estimator_state<DefaultImu9MahonyConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfRejectsStaleSamplesWithoutStateChanges)
{
    expect_stale_samples_preserve_estimator_state<DefaultImu9EkfConfig<double>>();
}

template<typename Config>
void expect_valid_gyro_is_not_flagged_when_propagation_is_skipped()
{
    OrientationEstimator<double, Config> estimator;
    for (double timestamp : {0.0, Config::max_dt_s * 2.0})
    {
        estimator.update(Imu9Sample<double>{timestamp, Vec<double, 3>(),
                                            Vec<double, 3>(0.0, 0.0, -constants::gravity<double>()),
                                            Vec<double, 3>(0.5, 0.0, 0.0)});
        EXPECT_TRUE(estimator.estimate().valid);
        EXPECT_FALSE(has_flag(estimator.estimate().flags, StatusFlag::gyro_rejected));
        EXPECT_EQ(has_flag(estimator.estimate().flags, StatusFlag::sample_time_invalid), timestamp > 0.0);
    }
}

TEST(SensorFusionOrientationTest, MahonySkippedPropagationDoesNotRejectValidGyro)
{
    expect_valid_gyro_is_not_flagged_when_propagation_is_skipped<DefaultImu9MahonyConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfSkippedPropagationDoesNotRejectValidGyro)
{
    expect_valid_gyro_is_not_flagged_when_propagation_is_skipped<DefaultImu9EkfConfig<double>>();
}

template<typename Config>
void expect_complete_dropout_is_invalid_and_preserves_state_time()
{
    OrientationEstimator<double, Config> estimator;
    const Vec<double, 3> gravity(0.0, 0.0, -constants::gravity<double>());
    const Vec<double, 3> magnetic_field(0.5, 0.0, 0.0);
    const auto update_sample =
        [&](double timestamp, const Vec<double, 3>& gyro, const Vec<double, 3>& accel, const Vec<double, 3>& mag)
    {
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            estimator.update(Imu9Sample<double>{timestamp, gyro, accel, mag});
        }
        else
        {
            estimator.update(Imu6Sample<double>{timestamp, gyro, accel});
        }
    };
    update_sample(0.0, Vec<double, 3>(), gravity, magnetic_field);
    update_sample(0.1, Vec<double, 3>(), gravity, magnetic_field);
    ASSERT_TRUE(estimator.estimate().valid);
    const Quaternion<double> before = estimator.orientation();
    const Vec<double, 3> bias_before = estimator.gyro_bias_estimate();
    const Vec<double, 3> invalid_gyro(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    for (double timestamp : {0.11, 0.12})
    {
        update_sample(timestamp, invalid_gyro, Vec<double, 3>(), Vec<double, 3>());
        EXPECT_FALSE(estimator.estimate().valid);
        EXPECT_EQ(estimator.estimate().quality, SolutionQuality::invalid);
        EXPECT_DOUBLE_EQ(estimator.estimate().confidence, 0.0);
        EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 0.1);
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::gyro_rejected));
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::accel_rejected));
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::orientation_held));
        EXPECT_LT(quaternion_distance(estimator.orientation(), before), 1e-6);
        EXPECT_EQ(estimator.gyro_bias_estimate(), bias_before);
        EXPECT_LT(quaternion_distance(estimator.predict_orientation(timestamp + 0.01, 0.1), before), 1e-6);
    }
    OrientationEstimator<double, Config> rebased = estimator;
    ASSERT_TRUE(rebased.rebase_timestamp(5.0));
    EXPECT_DOUBLE_EQ(rebased.estimate().timestamp_s, 4.98);
    EXPECT_FALSE(rebased.estimate().valid);
    update_sample(0.13, Vec<double, 3>(0.0, 0.0, 1.0), Vec<double, 3>(), Vec<double, 3>());
    EXPECT_TRUE(estimator.estimate().valid);
    EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 0.13);
    EXPECT_NEAR(quaternion_distance(estimator.orientation(), before), 0.01, 1e-5);
}

TEST(SensorFusionOrientationTest, MahonyCompleteDropoutRetainsStateTimeAndRecovers)
{
    expect_complete_dropout_is_invalid_and_preserves_state_time<DefaultImu6MahonyConfig<double>>();
    expect_complete_dropout_is_invalid_and_preserves_state_time<DefaultImu9MahonyConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfCompleteDropoutRetainsStateTimeAndRecovers)
{
    expect_complete_dropout_is_invalid_and_preserves_state_time<DefaultImu6EkfConfig<double>>();
    expect_complete_dropout_is_invalid_and_preserves_state_time<DefaultImu9EkfConfig<double>>();
}

template<typename Config>
void expect_accel_only_held_state_respects_confidence_policy()
{
    OrientationEstimator<double, Config> estimator;
    const Vec<double, 3> gravity(0.0, 0.0, -constants::gravity<double>());
    estimator.update(AccelSample<double>{1.0, gravity});
    ASSERT_TRUE(estimator.estimate().valid);
    estimator.update(AccelSample<double>{1.01, Vec<double, 3>()});
    EXPECT_EQ(estimator.estimate().valid, Config::held_orientation_confidence > 0.0);
    EXPECT_EQ(estimator.estimate().quality,
              Config::held_orientation_confidence > 0.0 ? SolutionQuality::degraded : SolutionQuality::invalid);
    EXPECT_DOUBLE_EQ(estimator.estimate().confidence, 0.95 * Config::held_orientation_confidence);
    EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 1.0);
    EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::orientation_held));
    estimator.update(AccelSample<double>{1.02, gravity});
    EXPECT_TRUE(estimator.estimate().valid);
    EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 1.02);
    EXPECT_FALSE(has_flag(estimator.estimate().flags, StatusFlag::orientation_held));
}

TEST(SensorFusionOrientationTest, AccelOnlyHeldStateRespectsConfidencePolicy)
{
    expect_accel_only_held_state_respects_confidence_policy<DefaultAccelOnlyConfig<double>>();
    expect_accel_only_held_state_respects_confidence_policy<HighDynamicsAccelOnlyConfig<double>>();
}

TEST(SensorFusionOrientationTest, TimestampRebasePreservesOrientationState)
{
    OrientationEstimator<float, DefaultImu6MahonyConfig<float>> estimator;
    const Vec<float, 3> gravity(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> yaw_rate(0.0F, 0.0F, 1.0F);

    estimator.update(Imu6Sample<float>{1000.0F, Vec<float, 3>(), gravity});
    estimator.update(Imu6Sample<float>{1000.1F, yaw_rate, gravity});
    const Quaternion<float> orientation_before_rebase = estimator.orientation();

    ASSERT_TRUE(estimator.rebase_timestamp(0.0F));
    EXPECT_NEAR(quaternion_distance(estimator.orientation(), orientation_before_rebase), 0.0F, 1e-6F);
    EXPECT_FLOAT_EQ(estimator.estimate().timestamp_s, 0.0F);

    estimator.update(Imu6Sample<float>{0.1F, yaw_rate, gravity});

    EXPECT_NEAR(quaternion_distance(estimator.orientation(), Quaternion<float>::identity()), 0.2F, 2e-2F);
    EXPECT_FALSE(estimator.rebase_timestamp(std::numeric_limits<float>::quiet_NaN()));
}

TEST(SensorFusionOrientationTest, Imu9MagRejectionFallsBackToHeadingWithDrift)
{
    OrientationEstimator<float, DefaultImu9MahonyConfig<float>> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(50.0F, 0.0F, 0.0F)});
    estimator.update(Imu9Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(5.0F, 0.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_FALSE(estimate.heading_referenced);
    EXPECT_EQ(estimate.observability, Observability::heading_with_drift);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::mag_rejected));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::propagation_only));
    EXPECT_GT(estimate.estimated_drift_rad, 0.0F);
}

TEST(SensorFusionOrientationTest, Imu9AcceptsMagnetometerSamplesInArbitraryConsistentUnits)
{
    OrientationEstimator<float, DefaultImu9MahonyConfig<float>> estimator;

    const Quaternion<float> orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), deg_to_rad_local(30.0F));

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(orientation),
                                       make_mag_body(orientation, 0.8F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_TRUE(estimate.heading_referenced);
    EXPECT_EQ(estimate.observability, Observability::full_3d);
    EXPECT_FALSE(has_flag(estimate.flags, StatusFlag::mag_rejected));
    EXPECT_NEAR(quaternion_distance(estimate.orientation, orientation), 0.0F, 1e-3F);
}

template<typename Config, typename Scalar = float>
void expect_equivalent_magnetic_units()
{
    const Quaternion<Scalar> target = Quaternion<Scalar>::from_euler(Scalar(0.2), Scalar(-0.1), Scalar(0.6));
    for (Scalar magnitude : {Scalar(50), Scalar(0.5), Scalar(0.00005), Scalar(1e-20), Scalar(1e20)})
    {
        OrientationEstimator<Scalar, Config> estimator;
        for (Scalar time : {Scalar(0), Scalar(0.01)})
        {
            estimator.update(
                Imu9Sample<Scalar>{time, Vec<Scalar, 3>(), make_accel_body(target), make_mag_body(target, magnitude)});
            EXPECT_TRUE(estimator.estimate().valid);
            EXPECT_TRUE(estimator.estimate().heading_referenced);
            EXPECT_EQ(estimator.estimate().observability, Observability::full_3d);
            EXPECT_FALSE(has_flag(estimator.estimate().flags, StatusFlag::mag_rejected));
            EXPECT_LT(quaternion_distance(estimator.orientation(), target), Scalar(0.003));
        }
    }
}

TEST(SensorFusionOrientationTest, EquivalentMagneticUnits)
{
    expect_equivalent_magnetic_units<DefaultImu9MahonyConfig<float>>();
    expect_equivalent_magnetic_units<DefaultImu9EkfConfig<float>>();
    expect_equivalent_magnetic_units<DefaultImu9MahonyConfig<double>, double>();
    expect_equivalent_magnetic_units<DefaultImu9EkfConfig<double>, double>();
}

template<typename Scalar>
void expect_scale_safe_direction_normalization()
{
    const Vec<Scalar, 3> fallback(0, 0, 1);
    const Scalar tolerance = Scalar(8) * std::numeric_limits<Scalar>::epsilon();
    const Scalar expected = Scalar(1) / std::sqrt(Scalar(2));
    for (Scalar scale : {std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::min(),
                         std::numeric_limits<Scalar>::denorm_min()})
    {
        if (!(scale > Scalar(0)))
        {
            continue;
        }
        const Vec<Scalar, 3> direction(scale, -scale, 0);
        EXPECT_TRUE(microla::fusion::detail::is_usable_direction(direction));
        const auto normalized = microla::fusion::detail::normalized_or(direction, fallback);
        EXPECT_NEAR(normalized[0], expected, tolerance);
        EXPECT_NEAR(normalized[1], -expected, tolerance);
        EXPECT_EQ(normalized[2], Scalar(0));
    }

    for (const Vec<Scalar, 3>& invalid :
         {Vec<Scalar, 3>(), Vec<Scalar, 3>(std::numeric_limits<Scalar>::quiet_NaN(), 0, 0),
          Vec<Scalar, 3>(std::numeric_limits<Scalar>::infinity(), 0, 0)})
    {
        EXPECT_FALSE(microla::fusion::detail::is_usable_direction(invalid));
        EXPECT_EQ(microla::fusion::detail::normalized_or(invalid, fallback), fallback);
    }
}

TEST(SensorFusionOrientationTest, ScaleSafeDirectionNormalization)
{
    expect_scale_safe_direction_normalization<float>();
    expect_scale_safe_direction_normalization<double>();
}

template<typename Config, typename Scalar>
struct CollinearMagneticReferenceConfig : Config
{
    static auto world_magnetic_reference() noexcept -> Vec<Scalar, 3> { return Vec<Scalar, 3>(0, 0, 1); }
};

template<typename Config, typename Scalar>
void expect_unobservable_magnetic_heading_is_rejected()
{
    const Quaternion<Scalar> target = Quaternion<Scalar>::from_euler(Scalar(0.2), Scalar(-0.1), Scalar(0.6));
    const Vec<Scalar, 3> gravity = make_accel_body(target);
    for (Scalar magnitude : {Scalar(50), Scalar(0.5), Scalar(0.00005)})
    {
        for (Scalar horizontal_ratio : {Scalar(0), Scalar(1e-10)})
        {
            for (bool heading_initialized : {false, true})
            {
                OrientationEstimator<Scalar, Config> estimator;
                if (heading_initialized)
                {
                    estimator.update(
                        Imu9Sample<Scalar>{Scalar(0), Vec<Scalar, 3>(), gravity, make_mag_body(target, magnitude)});
                    ASSERT_TRUE(estimator.estimate().heading_referenced);
                }
                const Quaternion<Scalar> before = estimator.orientation();
                const Vec<Scalar, 3> magnetic_field =
                    target.rotate_inverse(Vec<Scalar, 3>(magnitude * horizontal_ratio, 0, -magnitude));
                estimator.update(Imu9Sample<Scalar>{heading_initialized ? Scalar(0.01) : Scalar(0), Vec<Scalar, 3>(),
                                                    gravity, magnetic_field});
                EXPECT_TRUE(estimator.estimate().valid);
                EXPECT_FALSE(estimator.estimate().heading_referenced);
                EXPECT_EQ(estimator.estimate().observability, Observability::heading_with_drift);
                EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::mag_rejected));
                if (heading_initialized)
                {
                    EXPECT_LT(quaternion_distance(estimator.orientation(), before), Scalar(0.003));
                    estimator.update(
                        Imu9Sample<Scalar>{Scalar(0.02), Vec<Scalar, 3>(), Vec<Scalar, 3>(), magnetic_field});
                    EXPECT_TRUE(estimator.estimate().valid);
                    EXPECT_FALSE(estimator.estimate().heading_referenced);
                    EXPECT_EQ(estimator.estimate().observability, Observability::heading_with_drift);
                    EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::mag_rejected));
                    EXPECT_LT(quaternion_distance(estimator.orientation(), before), Scalar(0.003));
                }
            }
        }
    }

    OrientationEstimator<Scalar, CollinearMagneticReferenceConfig<Config, Scalar>> invalid_reference;
    invalid_reference.update(Imu9Sample<Scalar>{Scalar(0), Vec<Scalar, 3>(), gravity, make_mag_body(target)});
    EXPECT_FALSE(invalid_reference.estimate().valid);
    EXPECT_FALSE(invalid_reference.estimate().heading_referenced);
    EXPECT_TRUE(has_flag(invalid_reference.estimate().flags, StatusFlag::configuration_invalid));
}

TEST(SensorFusionOrientationTest, MahonyRejectsUnobservableMagneticHeading)
{
    expect_unobservable_magnetic_heading_is_rejected<DefaultImu9MahonyConfig<float>, float>();
    expect_unobservable_magnetic_heading_is_rejected<DefaultImu9MahonyConfig<double>, double>();
}

TEST(SensorFusionOrientationTest, EkfRejectsUnobservableMagneticHeading)
{
    expect_unobservable_magnetic_heading_is_rejected<DefaultImu9EkfConfig<float>, float>();
    expect_unobservable_magnetic_heading_is_rejected<DefaultImu9EkfConfig<double>, double>();
}

template<typename Config>
struct BoundedMagneticConfig : Config
{
    static constexpr bool enforce_mag_norm_limits = true;
};

template<typename Config>
void expect_startup_bad_mag_is_rejected_without_heading_lock()
{
    OrientationEstimator<float, BoundedMagneticConfig<Config>> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(5000.0F, 0.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_FALSE(estimate.heading_referenced);
    EXPECT_EQ(estimate.observability, Observability::heading_with_drift);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::mag_rejected));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::propagation_only));
}

TEST(SensorFusionOrientationTest, MahonyStartupRejectsDisturbedMagnetometerWithoutHeadingLock)
{
    expect_startup_bad_mag_is_rejected_without_heading_lock<DefaultImu9MahonyConfig<float>>();
}

TEST(SensorFusionOrientationTest, EkfStartupRejectsDisturbedMagnetometerWithoutHeadingLock)
{
    expect_startup_bad_mag_is_rejected_without_heading_lock<DefaultImu9EkfConfig<float>>();
}

TEST(SensorFusionOrientationTest, DirectionalMagneticDisturbanceIsRejectedAfterHeadingLock)
{
    OrientationEstimator<float, DefaultImu9MahonyConfig<float>> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(0.8F, 0.0F, 0.0F)});
    estimator.update(Imu9Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(-0.8F, 0.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_FALSE(estimate.heading_referenced);
    EXPECT_EQ(estimate.observability, Observability::heading_with_drift);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::mag_rejected));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::propagation_only));
}

template<typename Config>
void expect_measurement_feedback_reduces_orientation_error()
{
    const Vec<double, 3> gravity(0.0, 0.0, -constants::gravity<double>());
    const Vec<double, 3> magnetic_field(0.5, 0.0, 0.0);
    for (const Vec<double, 3>& axis :
         {Vec<double, 3>(1.0, 0.0, 0.0), Vec<double, 3>(0.0, 1.0, 0.0), Vec<double, 3>(0.0, 0.0, 1.0)})
    {
        OrientationEstimator<double, Config> estimator;
        estimator.update(Imu9Sample<double>{0.0, Vec<double, 3>(), gravity, magnetic_field});
        estimator.update(Imu9Sample<double>{0.1, axis * 0.5, gravity, magnetic_field});
        const double initial_error = quaternion_distance(estimator.orientation(), Quaternion<double>::identity());
        ASSERT_GT(initial_error, 1e-4);

        for (int step = 1; step <= 300; ++step)
        {
            estimator.update(Imu9Sample<double>{0.1 + 0.01 * step, Vec<double, 3>(), gravity, magnetic_field});
            ASSERT_TRUE(estimator.estimate().valid);
        }

        EXPECT_LT(quaternion_distance(estimator.orientation(), Quaternion<double>::identity()), initial_error * 0.5);
    }
}

TEST(SensorFusionOrientationTest, MahonyFeedbackReducesAttitudeErrorOnEachAxis)
{
    expect_measurement_feedback_reduces_orientation_error<DefaultImu9MahonyConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfFeedbackReducesAttitudeErrorOnEachAxis)
{
    expect_measurement_feedback_reduces_orientation_error<DefaultImu9EkfConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfNisRejectedMagnetometerFallsBackToHeadingWithDrift)
{
    OrientationEstimator<float, DefaultImu9EkfConfig<float>> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(50.0F, 0.0F, 0.0F)});
    estimator.update(Imu9Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(0.0F, -50.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_FALSE(estimate.heading_referenced);
    EXPECT_EQ(estimate.observability, Observability::heading_with_drift);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::mag_rejected));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::propagation_only));
}

template<typename Config>
void expect_gyro_dropout_uses_measurement_only_fallback()
{
    OrientationEstimator<float, Config> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(0.8F, 0.0F, 0.0F)});

    const Quaternion<float> target =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), deg_to_rad_local(45.0F));

    estimator.update(Imu9Sample<float>{0.1F, Vec<float, 3>(100.0F, 0.0F, 0.0F), make_accel_body(target),
                                       make_mag_body(target, 0.8F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_TRUE(estimate.heading_referenced);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::gyro_rejected));
    EXPECT_LT(quaternion_distance(estimate.orientation, target), deg_to_rad_local(35.0F));
    EXPECT_GT(quaternion_distance(estimate.orientation, Quaternion<float>::identity()), deg_to_rad_local(5.0F));
}

TEST(SensorFusionOrientationTest, MahonyGyroDropoutUsesMeasurementOnlyFallback)
{
    expect_gyro_dropout_uses_measurement_only_fallback<DefaultImu9MahonyConfig<float>>();
}

TEST(SensorFusionOrientationTest, EkfGyroDropoutUsesMeasurementOnlyFallback)
{
    expect_gyro_dropout_uses_measurement_only_fallback<DefaultImu9EkfConfig<float>>();
}

TEST(SensorFusionOrientationTest, EkfGyroDropoutHonorsNisGates)
{
    for (bool gyro_dropout : {true, false})
    {
        OrientationEstimator<double, StrictNisImu9EkfConfig<double>> estimator;
        const Quaternion<double> level = Quaternion<double>::identity();
        estimator.update(Imu9Sample<double>{0.0, Vec<double, 3>(), make_accel_body(level), make_mag_body(level, 0.5)});
        ASSERT_TRUE(estimator.estimate().valid);
        const Mat<double, 6, 6> covariance_before = estimator.error_covariance();
        const Vec<double, 3> bias_before = estimator.gyro_bias_estimate();
        const Quaternion<double> tilted = Quaternion<double>::from_euler(0.1, -0.1, 0.2);
        const Vec<double, 3> gyro =
            gyro_dropout ? Vec<double, 3>(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0) : Vec<double, 3>();
        estimator.update(
            Imu9Sample<double>{gyro_dropout ? 0.1 : 1.0, gyro, make_accel_body(tilted), make_mag_body(tilted, 0.55)});
        EXPECT_FALSE(estimator.estimate().valid);
        EXPECT_DOUBLE_EQ(estimator.estimate().timestamp_s, 0.0);
        EXPECT_EQ(has_flag(estimator.estimate().flags, StatusFlag::gyro_rejected), gyro_dropout);
        EXPECT_EQ(has_flag(estimator.estimate().flags, StatusFlag::sample_time_invalid), !gyro_dropout);
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::accel_rejected));
        EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::mag_rejected));
        EXPECT_LT(quaternion_distance(estimator.orientation(), level), 1e-6);
        EXPECT_EQ(estimator.gyro_bias_estimate(), bias_before);
        EXPECT_EQ(estimator.error_covariance(), covariance_before);
    }
}

template<typename T>
void expect_measurement_only_correction_resets_covariance()
{
    using Config = DefaultImu6EkfConfig<T>;
    OrientationEstimator<T, Config> estimator;
    const Vec<T, 3> gravity(T(0), T(0), T(-1));
    estimator.update(Imu6Sample<T>{T(0), Vec<T, 3>(), gravity * constants::gravity<T>()});
    const Mat<T, 6, 6> prior = estimator.error_covariance();
    const Quaternion<T> tilted = Quaternion<T>::from_euler(static_cast<T>(0.35), T(0), T(0));
    const Vec<T, 3> residual = tilted.rotate_inverse(gravity) - gravity;
    Mat<T, 3, 6> observation = Mat<T, 3, 6>::zero();
    const Mat<T, 3, 3> gravity_hat = microla::fusion::detail::skew(gravity);
    for (std::size_t row = 0; row < 3; ++row)
    {
        for (std::size_t col = 0; col < 3; ++col)
        {
            observation(row, col) = gravity_hat(row, col);
        }
    }
    const Mat<T, 3, 3> noise = Mat<T, 3, 3>::identity() * Config::r_accel;
    Mat<T, 3, 3> innovation_inverse;
    ASSERT_TRUE((observation * prior * observation.transpose() + noise).inverse(innovation_inverse));
    const Mat<T, 6, 3> gain = prior * observation.transpose() * innovation_inverse;
    const T correction = (gain * residual)[0];
    ASSERT_GT(correction, T(0));
    const Mat<T, 6, 6> remaining = Mat<T, 6, 6>::identity() - gain * observation;
    const Mat<T, 6, 6> joseph = remaining * prior * remaining.transpose() + gain * noise * gain.transpose();
    Mat<T, 6, 6> reset = Mat<T, 6, 6>::identity();
    reset(1, 1) = std::sin(correction) / correction;
    reset(2, 2) = reset(1, 1);
    reset(1, 2) = (T(1) - std::cos(correction)) / correction;
    reset(2, 1) = -reset(1, 2);
    const Mat<T, 6, 6> expected = reset * joseph * reset.transpose();

    estimator.update(Imu6Sample<T>{static_cast<T>(0.1), Vec<T, 3>(std::numeric_limits<T>::quiet_NaN(), T(0), T(0)),
                                   make_accel_body(tilted)});
    EXPECT_TRUE(estimator.estimate().valid);
    EXPECT_FALSE(has_flag(estimator.estimate().flags, StatusFlag::accel_rejected));
    EXPECT_LT(quaternion_distance(estimator.orientation(), tilted), static_cast<T>(0.35));
    for (std::size_t row = 0; row < 6; ++row)
    {
        for (std::size_t col = 0; col < 6; ++col)
        {
            EXPECT_NEAR(estimator.error_covariance()(row, col), expected(row, col),
                        T(100) * std::numeric_limits<T>::epsilon());
        }
    }
}

TEST(SensorFusionOrientationTest, EkfMeasurementOnlyCorrectionResetsCovariance)
{
    expect_measurement_only_correction_resets_covariance<float>();
    expect_measurement_only_correction_resets_covariance<double>();
}

template<typename T, typename Config>
void expect_repeated_corrections_keep_covariance_well_formed()
{
    OrientationEstimator<T, Config> estimator;
    const Quaternion<T> truth =
        Quaternion<T>::from_euler(static_cast<T>(0.15), static_cast<T>(-0.08), static_cast<T>(0.3));
    for (std::size_t step = 0; step < 200; ++step)
    {
        const T timestamp = static_cast<T>(step) * static_cast<T>(0.01);
        const Vec<T, 3> gyro = step % 7 == 0
                                   ? Vec<T, 3>(std::numeric_limits<T>::quiet_NaN(), T(0), T(0))
                                   : Vec<T, 3>(static_cast<T>(0.02), static_cast<T>(-0.01), static_cast<T>(0.03));
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            estimator.update(
                Imu9Sample<T>{timestamp, gyro, make_accel_body(truth), make_mag_body(truth, static_cast<T>(0.5))});
        }
        else
        {
            estimator.update(Imu6Sample<T>{timestamp, gyro, make_accel_body(truth)});
        }
        ASSERT_TRUE(estimator.estimate().valid) << step;
        const Mat<T, 6, 6>& covariance = estimator.error_covariance();
        ASSERT_TRUE(covariance.is_finite()) << step;
        for (std::size_t row = 0; row < 6; ++row)
        {
            for (std::size_t col = 0; col < 6; ++col)
            {
                EXPECT_NEAR(covariance(row, col), covariance(col, row), T(10) * std::numeric_limits<T>::epsilon());
            }
        }
        Mat<T, 6, 6> schur = covariance;
        for (std::size_t pivot = 0; pivot < 6; ++pivot)
        {
            ASSERT_GT(schur(pivot, pivot), T(0)) << step << ":" << pivot;
            for (std::size_t row = pivot + 1; row < 6; ++row)
            {
                for (std::size_t col = pivot + 1; col < 6; ++col)
                {
                    schur(row, col) -= schur(row, pivot) * schur(pivot, col) / schur(pivot, pivot);
                }
            }
        }
    }
}

TEST(SensorFusionOrientationTest, EkfRepeatedCorrectionsKeepCovarianceWellFormed)
{
    expect_repeated_corrections_keep_covariance_well_formed<float, DefaultImu6EkfConfig<float>>();
    expect_repeated_corrections_keep_covariance_well_formed<double, DefaultImu6EkfConfig<double>>();
    expect_repeated_corrections_keep_covariance_well_formed<float, DefaultImu9EkfConfig<float>>();
    expect_repeated_corrections_keep_covariance_well_formed<double, DefaultImu9EkfConfig<double>>();
}

template<typename Config>
void expect_gravity_only_fallback_preserves_unobserved_heading()
{
    OrientationEstimator<double, Config> estimator;
    const Quaternion<double> heading = Quaternion<double>::from_euler(0.0, 0.0, 0.8);
    const Vec<double, 3> gravity = make_accel_body(heading);
    if constexpr (Config::sensor_model == SensorModel::imu9)
    {
        estimator.update(Imu9Sample<double>{0.0, Vec<double, 3>(), gravity, make_mag_body(heading, 0.5)});
    }
    else
    {
        estimator.update(Imu6Sample<double>{0.0, Vec<double, 3>(), gravity});
        estimator.update(Imu6Sample<double>{0.1, Vec<double, 3>(0.0, 0.0, 8.0), gravity});
    }
    const Quaternion<double> before_dropout = estimator.orientation();
    const auto update_with_dropout = [&](double timestamp, const Vec<double, 3>& accel)
    {
        const Vec<double, 3> rejected_gyro(100.0, 0.0, 0.0);
        if constexpr (Config::sensor_model == SensorModel::imu9)
        {
            estimator.update(Imu9Sample<double>{timestamp, rejected_gyro, accel, Vec<double, 3>()});
        }
        else
        {
            estimator.update(Imu6Sample<double>{timestamp, rejected_gyro, accel});
        }
    };

    update_with_dropout(0.2, gravity);
    EXPECT_LT(quaternion_distance(estimator.orientation(), before_dropout), 1e-6);
    EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::gyro_rejected));
    EXPECT_FALSE(estimator.estimate().heading_referenced);
    EXPECT_EQ(estimator.estimate().observability, Observability::heading_with_drift);

    const Quaternion<double> tilted = Quaternion<double>::from_euler(0.2, -0.1, 0.8);
    const double tilt_error_before = quaternion_distance(estimator.orientation(), tilted);
    update_with_dropout(0.3, make_accel_body(tilted));
    EXPECT_LT(quaternion_distance(estimator.orientation(), tilted), tilt_error_before * 0.7);
}

TEST(SensorFusionOrientationTest, MahonyGravityOnlyFallbackPreservesUnobservedHeading)
{
    expect_gravity_only_fallback_preserves_unobserved_heading<DefaultImu6MahonyConfig<double>>();
    expect_gravity_only_fallback_preserves_unobserved_heading<DefaultImu9MahonyConfig<double>>();
}

TEST(SensorFusionOrientationTest, EkfGravityOnlyFallbackPreservesUnobservedHeading)
{
    expect_gravity_only_fallback_preserves_unobserved_heading<DefaultImu6EkfConfig<double>>();
    expect_gravity_only_fallback_preserves_unobserved_heading<DefaultImu9EkfConfig<double>>();
}

TEST(SensorFusionOrientationTest, CalibrationUpdateResetsStateAndAllowsMagReferenceRelearn)
{
    OrientationEstimator<float, DefaultImu9MahonyConfig<float>> estimator;

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(50.0F, 0.0F, 0.0F)});
    ASSERT_TRUE(estimator.estimate().valid);

    estimator.set_calibration(SensorCalibration<float>{});

    const OrientationEstimate<float>& after_reset = estimator.estimate();
    EXPECT_FALSE(after_reset.valid);
    EXPECT_EQ(after_reset.observability, Observability::none);

    estimator.update(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                       Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
                                       Vec<float, 3>(0.8F, 0.0F, 0.0F)});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_TRUE(estimate.heading_referenced);
    EXPECT_FALSE(has_flag(estimate.flags, StatusFlag::mag_rejected));
}

TEST(SensorFusionOrientationTest, InvalidCalibrationIsRejected)
{
    SensorCalibration<float> calibration;
    calibration.sensor_to_body = Quaternion<float>(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F);

    OrientationEstimator<float, DefaultAccelOnlyConfig<float>> estimator(calibration);
    estimator.update(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_FALSE(estimate.valid);
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::calibration_invalid));
    EXPECT_TRUE(has_flag(estimate.flags, StatusFlag::startup_not_initialized));
}

TEST(SensorFusionOrientationTest, NonUnitCalibrationQuaternionIsNormalized)
{
    SensorCalibration<float> calibration;
    calibration.sensor_to_body = Quaternion<float>(2.0F, 0.0F, 0.0F, 0.0F);

    OrientationEstimator<float, DefaultAccelOnlyConfig<float>> estimator(calibration);
    estimator.update(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const OrientationEstimate<float>& estimate = estimator.estimate();
    EXPECT_TRUE(estimate.valid);
    EXPECT_FALSE(has_flag(estimate.flags, StatusFlag::calibration_invalid));
}

TEST(SensorFusionTest, AccelOnlyTiltAngleTracksRelativeTilt)
{
    RelativeAngleEstimator<float, LeftAccelConfig<float>, RightAccelConfig<float>, TiltRelativeConfig<float>> estimator;

    estimator.update_left(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, -constants::gravity<float>(), 0.0F)});

    const RelativeAngleResult<float> result = estimator.compute();

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.primary.valid);
    EXPECT_EQ(result.primary.mode, PrimaryScalarOutput::tilt_angle);
    EXPECT_EQ(result.primary.observability, Observability::tilt_only);
    EXPECT_NEAR(result.primary.angle_rad, constants::half_pi<float>(), 1e-3F);
    EXPECT_NEAR(result.tilt_angle_rad, constants::half_pi<float>(), 1e-3F);
}

TEST(SensorFusionTest, RuntimeOutputOverrideRejectsUnobservableHeadingForAccelOnlyPair)
{
    RelativeAngleEstimator<float, LeftAccelConfig<float>, RightAccelConfig<float>, TiltRelativeConfig<float>> estimator;

    estimator.update_left(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const RelativeAngleResult<float> result = estimator.compute_scalar(PrimaryScalarOutput::heading_delta);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.primary.valid);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::heading_unobservable));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::output_not_supported));
}

TEST(SensorFusionTest, Imu6HeadingDeltaIsReturnedWithDriftMetadata)
{
    RelativeAngleEstimator<float, LeftImu6Config<float>, RightImu6Config<float>, HeadingRelativeConfig<float>>
        estimator;

    estimator.update_left(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                             Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    estimator.update_left(Imu6Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(Imu6Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 1.0F),
                                             Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.primary.valid);
    EXPECT_EQ(result.primary.observability, Observability::heading_with_drift);
    EXPECT_GT(result.primary.estimated_drift_rad, 0.0F);
    EXPECT_NEAR(result.primary.angle_rad, 0.1F, 2e-2F);
    EXPECT_TRUE(has_flag(result.right_entity.flags, StatusFlag::propagation_only));
}

TEST(SensorFusionTest, RelativeResultPropagatesEntityStatusFlags)
{
    RelativeAngleEstimator<float, DefaultImu9MahonyConfig<float>, DefaultImu9MahonyConfig<float>> estimator;

    const Vec<float, 3> gravity(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> magnetic_field(50.0F, 0.0F, 0.0F);
    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(), gravity, magnetic_field});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(), gravity, magnetic_field});
    estimator.update_left(Imu9Sample<float>{0.1F, Vec<float, 3>(), gravity, magnetic_field});
    estimator.update_right(Imu9Sample<float>{0.1F, Vec<float, 3>(100.0F, 0.0F, 0.0F), gravity, magnetic_field});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(has_flag(result.right_entity.flags, StatusFlag::gyro_rejected));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::gyro_rejected));
    EXPECT_EQ(result.primary.quality, SolutionQuality::degraded);
}

TEST(SensorFusionTest, Imu9EkfProvidesFull3DHeadingDelta)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>, HeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> left_orientation = Quaternion<float>::identity();
    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), deg_to_rad_local(45.0F));

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(left_orientation),
                                            make_mag_body(left_orientation)});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.primary.valid);
    EXPECT_EQ(result.primary.observability, Observability::full_3d);
    EXPECT_NEAR(result.primary.angle_rad, deg_to_rad_local(45.0F), 5e-2F);
    EXPECT_NEAR(result.heading_delta_rad, deg_to_rad_local(45.0F), 5e-2F);
}

TEST(SensorFusionTest, HingeTwistTracksPureTwistRotation)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>, HeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> left_orientation = Quaternion<float>::identity();
    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), constants::quarter_pi<float>());

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(left_orientation),
                                            make_mag_body(left_orientation)});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    const RelativeAngleResult<float> result = estimator.compute_scalar(PrimaryScalarOutput::hinge_twist);

    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.primary.angle_rad, constants::quarter_pi<float>(), 5e-2F);
    EXPECT_NEAR(result.hinge_twist_rad, constants::quarter_pi<float>(), 5e-2F);
    EXPECT_NEAR(result.swing_angle_rad, 0.0F, 5e-2F);
}

TEST(SensorFusionTest, SwingAngleTracksNonHingeRotation)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>, HeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> left_orientation = Quaternion<float>::identity();
    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(1.0F, 0.0F, 0.0F), deg_to_rad_local(20.0F));

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(left_orientation),
                                            make_mag_body(left_orientation)});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    const RelativeAngleResult<float> result = estimator.compute_scalar(PrimaryScalarOutput::swing_angle);

    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.primary.angle_rad, deg_to_rad_local(20.0F), 5e-2F);
    EXPECT_NEAR(result.swing_angle_rad, deg_to_rad_local(20.0F), 5e-2F);
    EXPECT_NEAR(result.hinge_twist_rad, 0.0F, 5e-2F);
}

TEST(SensorFusionTest, PairSkewAndStaleFlagsAreReported)
{
    RelativeAngleEstimator<float, LeftAccelConfig<float>, RightAccelConfig<float>, TiltRelativeConfig<float>> estimator;

    estimator.update_left(AccelSample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(AccelSample<float>{0.2F, Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::left_sample_stale));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::pair_time_skew_exceeded));
    EXPECT_EQ(result.primary.quality, SolutionQuality::degraded);
}

TEST(SensorFusionTest, RelativeHeadingCompensatesRecentPairSkewUsingEntityRates)
{
    RelativeAngleEstimator<float, LeftImu6Config<float>, RightImu6Config<float>,
                           SkewTolerantHeadingRelativeConfig<float>>
        estimator;

    estimator.update_left(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 1.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 1.0F),
                                             Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    estimator.update_left(Imu6Sample<float>{0.04F, Vec<float, 3>(0.0F, 0.0F, 1.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.primary.valid);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::pair_time_skew_exceeded));
    EXPECT_EQ(result.primary.observability, Observability::heading_with_drift);
    EXPECT_NEAR(result.primary.angle_rad, 0.0F, 1.5e-2F);
}

TEST(SensorFusionTest, DriftDisabledPolicyRejectsHeadingOnlyDriftResult)
{
    RelativeAngleEstimator<float, LeftImu6Config<float>, RightImu6Config<float>, NoDriftHeadingRelativeConfig<float>>
        estimator;

    estimator.update_left(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(Imu6Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                             Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    estimator.update_left(Imu6Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});
    estimator.update_right(Imu6Sample<float>{0.1F, Vec<float, 3>(0.0F, 0.0F, 1.0F),
                                             Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())});

    const RelativeAngleResult<float> result = estimator.compute_scalar(PrimaryScalarOutput::heading_delta);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.primary.valid);
    EXPECT_EQ(result.primary.observability, Observability::none);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::heading_unobservable));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::output_not_supported));
}

TEST(SensorFusionTest, ReferencePoseCanZeroRelativeOutput)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>,
                           ReferencedHeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), deg_to_rad_local(30.0F));

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            make_accel_body(Quaternion<float>::identity()),
                                            make_mag_body(Quaternion<float>::identity())});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    ASSERT_TRUE(estimator.capture_reference_pose());

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.primary.valid);
    EXPECT_NEAR(result.primary.angle_rad, 0.0F, 1e-5F);
    EXPECT_NEAR(result.heading_delta_rad, 0.0F, 1e-5F);
}

TEST(SensorFusionTest, ReferencePoseCanZeroRelativeTilt)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>,
                           ReferencedTiltRelativeConfig<float>>
        estimator;

    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(1.0F, 0.0F, 0.0F), deg_to_rad_local(20.0F));

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F),
                                            make_accel_body(Quaternion<float>::identity()),
                                            make_mag_body(Quaternion<float>::identity())});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    ASSERT_TRUE(estimator.capture_reference_pose());

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_TRUE(result.primary.valid);
    EXPECT_NEAR(result.primary.angle_rad, 0.0F, 1e-5F);
}

TEST(SensorFusionTest, CalibrationChangeClearsCapturedReferencePose)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>,
                           ReferencedHeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), deg_to_rad_local(30.0F));
    const Imu9Sample<float> left_sample{0.0F, Vec<float, 3>(), make_accel_body(Quaternion<float>::identity()),
                                        make_mag_body(Quaternion<float>::identity())};
    const Imu9Sample<float> right_sample{0.0F, Vec<float, 3>(), make_accel_body(right_orientation),
                                         make_mag_body(right_orientation)};

    estimator.update_left(left_sample);
    estimator.update_right(right_sample);
    ASSERT_TRUE(estimator.capture_reference_pose());

    estimator.set_left_calibration(SensorCalibration<float>{});
    estimator.update_left(left_sample);

    const RelativeAngleResult<float> result = estimator.compute();
    EXPECT_TRUE(result.primary.valid);
    EXPECT_FALSE(result.reference_active);
    EXPECT_NEAR(result.primary.angle_rad, deg_to_rad_local(30.0F), 5e-2F);
}

TEST(SensorFusionTest, MismatchedWorldFramesAreRejected)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, MismatchedWorldImu9Config<float>,
                           HeadingRelativeConfig<float>>
        estimator;

    const Quaternion<float> orientation = Quaternion<float>::identity();
    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(orientation),
                                            make_mag_body(orientation)});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(orientation),
                                             make_mag_body(orientation)});

    const RelativeAngleResult<float> result = estimator.compute();

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.primary.valid);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::configuration_invalid));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::output_not_supported));
}

TEST(SensorFusionTest, InconsistentRightHingeAxisIsRejected)
{
    RelativeAngleEstimator<float, LeftImu9EkfConfig<float>, RightImu9EkfConfig<float>,
                           InconsistentHingeRelativeConfig<float>>
        estimator;

    const Quaternion<float> left_orientation = Quaternion<float>::identity();
    const Quaternion<float> right_orientation =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), constants::quarter_pi<float>());

    estimator.update_left(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(left_orientation),
                                            make_mag_body(left_orientation)});
    estimator.update_right(Imu9Sample<float>{0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), make_accel_body(right_orientation),
                                             make_mag_body(right_orientation)});

    const RelativeAngleResult<float> result = estimator.compute_scalar(PrimaryScalarOutput::hinge_twist);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.primary.valid);
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::configuration_invalid));
    EXPECT_TRUE(has_flag(result.primary.flags, StatusFlag::output_not_supported));
}
