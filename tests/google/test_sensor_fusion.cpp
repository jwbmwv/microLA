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
struct NoDriftHeadingRelativeConfig : DefaultRelativeAngleConfig<T>
{
    static constexpr bool allow_drift_with_quality_flags = false;
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::tilt_angle;
};

}  // namespace

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

template<typename Config>
void expect_startup_bad_mag_is_rejected_without_heading_lock()
{
    OrientationEstimator<float, Config> estimator;

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
