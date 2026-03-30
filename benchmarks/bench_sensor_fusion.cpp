// SPDX-License-Identifier: Apache-2.0
// Sensor-fusion pipeline benchmarks
#include <benchmark/benchmark.h>

#include <microla/sensor_fusion.hpp>

using namespace microla;
using namespace microla::fusion;

namespace
{

using MahonyConfig = DefaultImu9MahonyConfig<float>;
using EkfConfig = DefaultImu9EkfConfig<float>;
using Imu6Config = DefaultImu6MahonyConfig<float>;

struct ReferencedHeadingConfig : DefaultRelativeAngleConfig<float>
{
    static constexpr bool apply_reference_pose = true;
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::heading_delta;
};

struct SkewAlignedHeadingConfig : DefaultRelativeAngleConfig<float>
{
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::heading_delta;
    static constexpr float min_confidence_to_publish = 0.2F;
};

struct MotionState
{
    float timestamp_s = 0.0F;
    Quaternion<float> orientation = Quaternion<float>::identity();
};

struct PreparedCalibration
{
    SensorCalibration<float> calibration{};
    Vec<float, 3> accel_inverse_scale = Vec<float, 3>::one();
    Vec<float, 3> gyro_inverse_scale = Vec<float, 3>::one();
    Mat<float, 3, 3> mag_soft_iron_inverse = Mat<float, 3, 3>::identity();
};

template<typename T>
auto gravity_world() noexcept -> Vec<T, 3>
{
    return Vec<T, 3>(T(0), T(0), -constants::gravity<T>());
}

template<typename T>
auto magnetic_world() noexcept -> Vec<T, 3>
{
    return Vec<T, 3>(static_cast<T>(50.0), T(0), T(0));
}

template<typename T>
auto make_accel_body(const Quaternion<T>& body_to_world) noexcept -> Vec<T, 3>
{
    return body_to_world.rotate_inverse(gravity_world<T>());
}

template<typename T>
auto make_mag_body(const Quaternion<T>& body_to_world) noexcept -> Vec<T, 3>
{
    return body_to_world.rotate_inverse(magnetic_world<T>());
}

auto make_prepared_calibration() noexcept -> PreparedCalibration
{
    PreparedCalibration prepared;
    prepared.calibration.sensor_to_body =
        Quaternion<float>::from_axis_angle(Vec<float, 3>::unit_z(), 0.12F).normalized();
    prepared.calibration.accel_bias = Vec<float, 3>(0.03F, -0.02F, 0.01F);
    prepared.calibration.accel_scale = Vec<float, 3>(1.01F, 0.99F, 1.02F);
    prepared.calibration.gyro_bias = Vec<float, 3>(0.001F, -0.002F, 0.0015F);
    prepared.calibration.gyro_scale = Vec<float, 3>(1.005F, 0.995F, 1.003F);
    prepared.calibration.mag_bias = Vec<float, 3>(0.6F, -0.4F, 0.25F);
    prepared.calibration.mag_soft_iron = Mat<float, 3, 3>::identity();
    prepared.calibration.mag_soft_iron(0, 0) = 1.04F;
    prepared.calibration.mag_soft_iron(0, 1) = 0.01F;
    prepared.calibration.mag_soft_iron(1, 0) = 0.01F;
    prepared.calibration.mag_soft_iron(1, 1) = 0.98F;
    prepared.calibration.mag_soft_iron(2, 2) = 1.02F;

    prepared.accel_inverse_scale =
        Vec<float, 3>(1.0F / prepared.calibration.accel_scale[0], 1.0F / prepared.calibration.accel_scale[1],
                      1.0F / prepared.calibration.accel_scale[2]);
    prepared.gyro_inverse_scale =
        Vec<float, 3>(1.0F / prepared.calibration.gyro_scale[0], 1.0F / prepared.calibration.gyro_scale[1],
                      1.0F / prepared.calibration.gyro_scale[2]);

    if (!prepared.calibration.mag_soft_iron.inverse(prepared.mag_soft_iron_inverse))
    {
        prepared.mag_soft_iron_inverse = Mat<float, 3, 3>::identity();
    }

    return prepared;
}

auto apply_linear_calibration_inverse(const Vec<float, 3>& body_value, const Quaternion<float>& sensor_to_body,
                                      const Vec<float, 3>& bias,
                                      const Vec<float, 3>& inverse_scale) noexcept -> Vec<float, 3>
{
    const Vec<float, 3> sensor_value = sensor_to_body.inverse_unit().rotate(body_value);
    return bias + sensor_value.hadamard(inverse_scale);
}

auto apply_mag_calibration_inverse(const Vec<float, 3>& body_value,
                                   const PreparedCalibration& prepared) noexcept -> Vec<float, 3>
{
    const Vec<float, 3> sensor_value = prepared.calibration.sensor_to_body.inverse_unit().rotate(body_value);
    return prepared.calibration.mag_bias + prepared.mag_soft_iron_inverse * sensor_value;
}

void advance_motion(MotionState& motion, const Vec<float, 3>& gyro_body, float dt) noexcept
{
    const float omega = gyro_body.length();
    if (omega > 0.0F)
    {
        motion.orientation =
            (motion.orientation * Quaternion<float>::from_axis_angle(gyro_body / omega, omega * dt)).normalized();
    }
    motion.timestamp_s += dt;
}

auto make_identity_sample(const MotionState& motion, const Vec<float, 3>& gyro_body) noexcept -> Imu9Sample<float>
{
    return Imu9Sample<float>{motion.timestamp_s, gyro_body, make_accel_body(motion.orientation),
                             make_mag_body(motion.orientation)};
}

auto make_imu6_sample(const MotionState& motion, const Vec<float, 3>& gyro_body) noexcept -> Imu6Sample<float>
{
    return Imu6Sample<float>{motion.timestamp_s, gyro_body, make_accel_body(motion.orientation)};
}

auto make_calibrated_sample(const MotionState& motion, const Vec<float, 3>& gyro_body,
                            const PreparedCalibration& prepared) noexcept -> Imu9Sample<float>
{
    const Vec<float, 3> accel_body = make_accel_body(motion.orientation);
    const Vec<float, 3> mag_body = make_mag_body(motion.orientation);

    return Imu9Sample<float>{
        motion.timestamp_s,
        apply_linear_calibration_inverse(gyro_body, prepared.calibration.sensor_to_body, prepared.calibration.gyro_bias,
                                         prepared.gyro_inverse_scale),
        apply_linear_calibration_inverse(accel_body, prepared.calibration.sensor_to_body,
                                         prepared.calibration.accel_bias, prepared.accel_inverse_scale),
        apply_mag_calibration_inverse(mag_body, prepared)};
}

static void bm_sensor_fusion_mahony_identity_update(benchmark::State& state)
{
    constexpr float dt = 0.01F;
    const Vec<float, 3> gyro_body(0.12F, -0.04F, 0.35F);
    MotionState motion;
    OrientationEstimator<float, MahonyConfig> estimator;
    estimator.update(make_identity_sample(motion, Vec<float, 3>::zero()));

    for (auto _ : state)
    {
        state.PauseTiming();
        advance_motion(motion, gyro_body, dt);
        const Imu9Sample<float> sample = make_identity_sample(motion, gyro_body);
        state.ResumeTiming();

        estimator.update(sample);
        benchmark::DoNotOptimize(estimator.orientation());
        benchmark::DoNotOptimize(static_cast<float>(estimator.estimate().confidence));
    }
}
BENCHMARK(bm_sensor_fusion_mahony_identity_update)->Unit(benchmark::kMicrosecond);

static void bm_sensor_fusion_mahony_runtime_calibration_update(benchmark::State& state)
{
    constexpr float dt = 0.01F;
    const Vec<float, 3> gyro_body(0.12F, -0.04F, 0.35F);
    const PreparedCalibration prepared = make_prepared_calibration();
    MotionState motion;
    OrientationEstimator<float, MahonyConfig> estimator;
    estimator.set_calibration(prepared.calibration);
    estimator.update(make_calibrated_sample(motion, Vec<float, 3>::zero(), prepared));

    for (auto _ : state)
    {
        state.PauseTiming();
        advance_motion(motion, gyro_body, dt);
        const Imu9Sample<float> sample = make_calibrated_sample(motion, gyro_body, prepared);
        state.ResumeTiming();

        estimator.update(sample);
        benchmark::DoNotOptimize(estimator.orientation());
        benchmark::DoNotOptimize(static_cast<float>(estimator.estimate().confidence));
    }
}
BENCHMARK(bm_sensor_fusion_mahony_runtime_calibration_update)->Unit(benchmark::kMicrosecond);

static void bm_sensor_fusion_ekf_update(benchmark::State& state)
{
    constexpr float dt = 0.01F;
    const Vec<float, 3> gyro_body(0.08F, 0.02F, 0.25F);
    MotionState motion;
    OrientationEstimator<float, EkfConfig> estimator;
    estimator.update(make_identity_sample(motion, Vec<float, 3>::zero()));

    for (auto _ : state)
    {
        state.PauseTiming();
        advance_motion(motion, gyro_body, dt);
        const Imu9Sample<float> sample = make_identity_sample(motion, gyro_body);
        state.ResumeTiming();

        estimator.update(sample);
        benchmark::DoNotOptimize(estimator.orientation());
        benchmark::DoNotOptimize(static_cast<float>(estimator.estimate().confidence));
    }
}
BENCHMARK(bm_sensor_fusion_ekf_update)->Unit(benchmark::kMicrosecond);

static void bm_sensor_fusion_relative_compute(benchmark::State& state)
{
    constexpr float dt = 0.01F;
    const Vec<float, 3> left_gyro(0.05F, 0.01F, 0.18F);
    const Vec<float, 3> right_gyro(-0.03F, 0.02F, 0.32F);
    MotionState left_motion;
    MotionState right_motion;
    RelativeAngleEstimator<float, MahonyConfig, MahonyConfig, ReferencedHeadingConfig> estimator;

    estimator.update_left(make_identity_sample(left_motion, Vec<float, 3>::zero()));
    estimator.update_right(make_identity_sample(right_motion, Vec<float, 3>::zero()));
    benchmark::DoNotOptimize(estimator.capture_reference_pose());

    for (auto _ : state)
    {
        state.PauseTiming();
        advance_motion(left_motion, left_gyro, dt);
        advance_motion(right_motion, right_gyro, dt);
        const Imu9Sample<float> left_sample = make_identity_sample(left_motion, left_gyro);
        const Imu9Sample<float> right_sample = make_identity_sample(right_motion, right_gyro);
        state.ResumeTiming();

        estimator.update_left(left_sample);
        estimator.update_right(right_sample);
        benchmark::DoNotOptimize(estimator.compute());
    }
}
BENCHMARK(bm_sensor_fusion_relative_compute)->Unit(benchmark::kMicrosecond);

static void bm_sensor_fusion_relative_compute_with_alignment(benchmark::State& state)
{
    constexpr float dt = 0.04F;
    const Vec<float, 3> shared_gyro(0.0F, 0.0F, 0.75F);
    MotionState left_motion;
    MotionState right_motion;
    RelativeAngleEstimator<float, Imu6Config, Imu6Config, SkewAlignedHeadingConfig> estimator;

    estimator.update_left(make_imu6_sample(left_motion, shared_gyro));
    estimator.update_right(make_imu6_sample(right_motion, shared_gyro));
    Imu6Sample<float> lagged_right_sample = make_imu6_sample(right_motion, shared_gyro);

    for (auto _ : state)
    {
        state.PauseTiming();
        advance_motion(left_motion, shared_gyro, dt);
        advance_motion(right_motion, shared_gyro, dt);
        const Imu6Sample<float> left_sample = make_imu6_sample(left_motion, shared_gyro);
        const Imu6Sample<float> next_right_sample = make_imu6_sample(right_motion, shared_gyro);
        state.ResumeTiming();

        estimator.update_left(left_sample);
        estimator.update_right(lagged_right_sample);
        benchmark::DoNotOptimize(estimator.compute());

        lagged_right_sample = next_right_sample;
    }
}
BENCHMARK(bm_sensor_fusion_relative_compute_with_alignment)->Unit(benchmark::kMicrosecond);

}  // namespace

BENCHMARK_MAIN();
