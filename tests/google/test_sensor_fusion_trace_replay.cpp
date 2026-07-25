// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "sensor_fusion_trace_fixture.hpp"

#include <microla/sensor_fusion.hpp>

#include <cmath>

using namespace microla;
using namespace microla::fusion;

namespace
{

template<typename T>
auto quaternion_distance(const Quaternion<T>& lhs, const Quaternion<T>& rhs) noexcept -> T
{
    const Quaternion<T> delta = (lhs.inverse_unit() * rhs).normalized();
    return T(2) * std::acos(std::min(T(1), std::abs(delta.w())));
}

template<typename T>
struct ReferencedHeadingTraceConfig : DefaultRelativeAngleConfig<T>
{
    static constexpr bool apply_reference_pose = true;
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::heading_delta;
};

}  // namespace

TEST(SensorFusionTraceReplay, Imu9EkfLifecycleFlagsRemainStable)
{
    OrientationEstimator<float, DefaultImu9EkfConfig<float>> estimator;

    for (const test_fixture::Imu9TraceFrame& frame : test_fixture::imu9_ekf_lifecycle_trace)
    {
        estimator.update(
            Imu9Sample<float>{frame.timestamp_s, frame.gyro_rad_s, frame.accel_m_s2, frame.magnetic_field});

        const OrientationEstimate<float>& estimate = estimator.estimate();
        ASSERT_TRUE(estimate.valid) << "timestamp " << frame.timestamp_s;
        if (frame.required_primary_flag != StatusFlag::none)
        {
            EXPECT_TRUE(has_flag(estimate.flags, frame.required_primary_flag)) << "timestamp " << frame.timestamp_s;
        }
        if (frame.required_secondary_flag != StatusFlag::none)
        {
            EXPECT_TRUE(has_flag(estimate.flags, frame.required_secondary_flag)) << "timestamp " << frame.timestamp_s;
        }

        if (frame.timestamp_s == 0.2F)
        {
            EXPECT_FALSE(estimate.heading_referenced);
            EXPECT_EQ(estimate.observability, Observability::heading_with_drift);
        }
    }
}

TEST(SensorFusionTraceReplay, TimingReplayRejectsReorderedSamplesAndSupportsRebase)
{
    OrientationEstimator<float, DefaultImu6MahonyConfig<float>> estimator;
    const auto& initial = test_fixture::imu6_timing_trace[0];
    const auto& reordered = test_fixture::imu6_timing_trace[1];
    const auto& rebased = test_fixture::imu6_timing_trace[2];

    estimator.update(Imu6Sample<float>{initial.timestamp_s, initial.gyro_rad_s, initial.accel_m_s2});
    ASSERT_TRUE(estimator.estimate().valid);
    const Quaternion<float> orientation_before_reorder = estimator.orientation();

    estimator.update(Imu6Sample<float>{reordered.timestamp_s, reordered.gyro_rad_s, reordered.accel_m_s2});
    EXPECT_FALSE(estimator.estimate().valid);
    EXPECT_TRUE(has_flag(estimator.estimate().flags, StatusFlag::sample_time_invalid));
    EXPECT_NEAR(quaternion_distance(estimator.orientation(), orientation_before_reorder), 0.0F, 1e-6F);

    estimator.rebase_timestamp(0.0F);
    EXPECT_NEAR(quaternion_distance(estimator.orientation(), orientation_before_reorder), 0.0F, 1e-6F);
    estimator.update(Imu6Sample<float>{rebased.timestamp_s, rebased.gyro_rad_s, rebased.accel_m_s2});
    EXPECT_TRUE(estimator.estimate().valid);
}

TEST(SensorFusionTraceReplay, PairedReplayReportsSkewAndInvalidatesReferenceAfterCalibration)
{
    using Estimator = RelativeAngleEstimator<float, DefaultImu9MahonyConfig<float>, DefaultImu9MahonyConfig<float>,
                                             ReferencedHeadingTraceConfig<float>>;

    Estimator estimator;
    const Vec<float, 3> gyro(0.0F, 0.0F, 1.0F);
    const Vec<float, 3> gravity(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> magnetic_field(50.0F, 0.0F, 0.0F);

    estimator.update_left(Imu9Sample<float>{0.0F, gyro, gravity, magnetic_field});
    estimator.update_right(Imu9Sample<float>{0.0F, gyro, gravity, magnetic_field});
    ASSERT_TRUE(estimator.capture_reference_pose());

    estimator.update_left(Imu9Sample<float>{0.04F, gyro, gravity, magnetic_field});
    const RelativeAngleResult<float> skewed_result = estimator.compute();
    ASSERT_TRUE(skewed_result.valid);
    EXPECT_TRUE(has_flag(skewed_result.primary.flags, StatusFlag::pair_time_skew_exceeded));

    estimator.set_left_calibration(SensorCalibration<float>{});
    estimator.update_left(Imu9Sample<float>{0.05F, gyro, gravity, magnetic_field});
    const RelativeAngleResult<float> recalibrated_result = estimator.compute();
    EXPECT_FALSE(recalibrated_result.reference_active);
}