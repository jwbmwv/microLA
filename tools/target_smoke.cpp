// SPDX-License-Identifier: Apache-2.0

#include <microla/sensor_fusion.hpp>

auto microla_target_mahony_step(float timestamp_s) noexcept -> float
{
    using namespace microla;
    using namespace microla::fusion;

    OrientationEstimator<float, DefaultImu9MahonyConfig<float>> estimator;
    const Vec<float, 3> gyro(0.0F, 0.0F, 0.2F);
    const Vec<float, 3> accel(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> mag(50.0F, 0.0F, 0.0F);

    estimator.update(Imu9Sample<float>{timestamp_s, gyro, accel, mag});
    estimator.update(Imu9Sample<float>{timestamp_s + 0.001F, gyro, accel, mag});
    return estimator.orientation().w();
}

auto microla_target_mekf_step(float timestamp_s) noexcept -> float
{
    using namespace microla;
    using namespace microla::fusion;

    OrientationEstimator<float, DefaultImu9EkfConfig<float>> estimator;
    const Vec<float, 3> gyro(0.0F, 0.0F, 0.2F);
    const Vec<float, 3> accel(0.0F, 0.0F, -constants::gravity<float>());
    const Vec<float, 3> mag(50.0F, 0.0F, 0.0F);

    estimator.update(Imu9Sample<float>{timestamp_s, gyro, accel, mag});
    estimator.update(Imu9Sample<float>{timestamp_s + 0.001F, gyro, accel, mag});
    return estimator.orientation().w();
}