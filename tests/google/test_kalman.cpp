// SPDX-License-Identifier: Apache-2.0
/// @file test_kalman.cpp
/// @brief Unit tests for Kalman filter and Extended Kalman filter
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <gtest/gtest.h>
#include <microla/kalman.hpp>
#include <microla/extended_kalman.hpp>
#include <cmath>

using namespace microla;

TEST(KalmanFilterTest, FailedPredictionPreservesBothStateAndCovariance)
{
    KalmanFilter<double, 1, 1> filter;
    filter.set_state(Vec<double, 1>(std::numeric_limits<double>::max()));
    filter.set_state_transition(Mat<double, 1, 1>{2});
    const auto covariance = filter.get_covariance();
    filter.predict();
    EXPECT_DOUBLE_EQ(filter.get_state()[0], std::numeric_limits<double>::max());
    EXPECT_EQ(filter.get_covariance(), covariance);
    filter.set_state(Vec<double, 1>(1));
    filter.predict(Vec<double, 1>(std::numeric_limits<double>::infinity()), Mat<double, 1, 1>{1});
    EXPECT_DOUBLE_EQ(filter.get_state()[0], 1.0);
    EXPECT_EQ(filter.get_covariance(), covariance);
    filter.set_state_transition(Mat<double, 1, 1>{std::numeric_limits<double>::max()});
    filter.predict();
    EXPECT_DOUBLE_EQ(filter.get_state()[0], 1.0);
    EXPECT_EQ(filter.get_covariance(), covariance);
}

TEST(ExtendedKalmanFilterTest, FailedPredictionPreservesBothStateAndCovariance)
{
    using State = Vec<double, 1>;
    using Matrix = Mat<double, 1, 1>;
    ExtendedKalmanFilter<double, 1, 1> filter;
    const auto covariance = filter.get_covariance();
    filter.set_state_transition(
        +[](const State&, double) -> State { return {std::numeric_limits<double>::quiet_NaN()}; },
        +[](const State&, double) { return Matrix::identity(); });
    filter.predict(0.1);
    EXPECT_DOUBLE_EQ(filter.get_state()[0], 0.0);
    EXPECT_EQ(filter.get_covariance(), covariance);
    filter.set_state_transition(
        +[](const State& state, double) { return state + State(1); },
        +[](const State&, double) { return Matrix{std::numeric_limits<double>::max()}; });
    filter.predict(0.1);
    EXPECT_DOUBLE_EQ(filter.get_state()[0], 0.0);
    EXPECT_EQ(filter.get_covariance(), covariance);
    filter.predict(std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(filter.get_state()[0], 0.0);
}

// ==================== Standard Kalman Filter Tests ====================

TEST(KalmanFilterTest, DefaultConstruction)
{
    KalmanFilter<float, 2, 1> kf;

    // State should be zero-initialized
    EXPECT_FLOAT_EQ(kf.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(kf.get_state(1), 0.0F);

    // Covariance should be identity
    EXPECT_FLOAT_EQ(kf.get_variance(0), 1.0F);
    EXPECT_FLOAT_EQ(kf.get_variance(1), 1.0F);
}

TEST(KalmanFilterTest, StateInitialization)
{
    KalmanFilter<float, 3, 2> kf;

    Vec<float, 3> x0({1.0F, 2.0F, 3.0F});
    Mat<float, 3, 3> P0 = Mat<float, 3, 3>::identity() * 2.0F;

    kf.reset(x0, P0);

    EXPECT_FLOAT_EQ(kf.get_state(0), 1.0F);
    EXPECT_FLOAT_EQ(kf.get_state(1), 2.0F);
    EXPECT_FLOAT_EQ(kf.get_state(2), 3.0F);
    EXPECT_FLOAT_EQ(kf.get_variance(0), 2.0F);
}

TEST(KalmanFilterTest, PredictStep)
{
    KalmanFilter<float, 2, 1> kf;

    // Set initial state
    Vec<float, 2> x0({0.0F, 1.0F});  // position=0, velocity=1
    kf.set_state(x0);

    // State transition: position += velocity * dt
    float dt = 0.1F;
    Mat<float, 2, 2> F({1.0F, dt, 0.0F, 1.0F});
    kf.set_state_transition(F);

    // Predict
    kf.predict();

    // After prediction: position should be 0 + 1*0.1 = 0.1
    EXPECT_NEAR(kf.get_state(0), 0.1F, 1e-6F);
    EXPECT_NEAR(kf.get_state(1), 1.0F, 1e-6F);
}

TEST(KalmanFilterTest, UpdateStep)
{
    KalmanFilter<float, 2, 1> kf;

    // Initial state: position=0, velocity=1
    Vec<float, 2> x0({0.0F, 1.0F});
    Mat<float, 2, 2> P0 = Mat<float, 2, 2>::identity();
    kf.reset(x0, P0);

    // Measurement matrix (measure position only)
    Mat<float, 1, 2> H({1.0F, 0.0F});
    kf.set_measurement_matrix(H);

    // Set reasonable noise
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});
    kf.set_process_noise(Q);
    kf.set_measurement_noise(R);

    // Measurement: position = 0.5
    Vec<float, 1> z({0.5F});
    bool success = kf.update(z);

    EXPECT_TRUE(success);
    // State should move toward measurement
    EXPECT_GT(kf.get_state(0), 0.0F);
    EXPECT_LT(kf.get_state(0), 0.5F);
}

TEST(KalmanFilterTest, ConstantVelocityTracking)
{
    KalmanFilter<float, 2, 1> kf;

    float dt = 0.1F;
    Mat<float, 2, 2> F({1.0F, dt, 0.0F, 1.0F});
    Mat<float, 1, 2> H({1.0F, 0.0F});
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001F;
    Mat<float, 1, 1> R({0.1F});

    kf.set_state_transition(F);
    kf.set_measurement_matrix(H);
    kf.set_process_noise(Q);
    kf.set_measurement_noise(R);

    // Initial state
    kf.reset(Vec<float, 2>({0.0F, 1.0F}), Mat<float, 2, 2>::identity());

    // Simulate 10 steps with constant velocity
    for (int i = 0; i < 10; ++i)
    {
        float true_pos = 1.0F * i * dt;

        kf.predict();
        kf.update(Vec<float, 1>({true_pos}));
    }

    // Velocity estimate should converge close to 1.0
    EXPECT_NEAR(kf.get_state(1), 1.0F, 0.2F);
}

TEST(KalmanFilterTest, PredictWithControl)
{
    KalmanFilter<float, 2, 1> kf;

    Vec<float, 2> x0({0.0F, 0.0F});
    kf.set_state(x0);

    float dt = 0.1F;
    Mat<float, 2, 2> F({1.0F, dt, 0.0F, 1.0F});
    Mat<float, 2, 1> B({0.5F * dt * dt, dt});  // Control affects position and velocity

    kf.set_state_transition(F);

    // Control input: acceleration = 2.0
    Vec<float, 1> u({2.0F});
    kf.predict<1>(u, B);

    // Check state after control input
    // position = 0.5 * accel * dt^2 = 0.5 * 2.0 * 0.01 = 0.01
    // velocity = accel * dt = 2.0 * 0.1 = 0.2
    EXPECT_NEAR(kf.get_state(0), 0.01F, 1e-5F);
    EXPECT_NEAR(kf.get_state(1), 0.2F, 1e-5F);
}

TEST(KalmanFilterTest, Innovation)
{
    KalmanFilter<float, 2, 1> kf;

    kf.set_state(Vec<float, 2>({1.0F, 0.5F}));

    Mat<float, 1, 2> H({1.0F, 0.0F});
    kf.set_measurement_matrix(H);

    // Innovation should be measurement - prediction
    Vec<float, 1> z({2.0F});
    Vec<float, 1> innovation = kf.compute_innovation(z);

    EXPECT_NEAR(innovation[0], 1.0F, 1e-6F);  // 2.0 - 1.0 = 1.0
}

TEST(KalmanFilterTest, NIS)
{
    KalmanFilter<float, 2, 1> kf;

    kf.set_state(Vec<float, 2>({1.0F, 0.5F}));

    Mat<float, 1, 2> H({1.0F, 0.0F});
    Mat<float, 1, 1> R({0.1F});
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);

    // NIS should be positive for any measurement
    Vec<float, 1> z({2.0F});
    float nis = kf.compute_nis(z);

    EXPECT_GT(nis, 0.0F);
}

TEST(KalmanFilterTest, Reset)
{
    KalmanFilter<float, 2, 1> kf;

    // Set some state
    kf.set_state(Vec<float, 2>({5.0F, 3.0F}));

    // Reset
    kf.reset();

    // Should return to zero state
    EXPECT_FLOAT_EQ(kf.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(kf.get_state(1), 0.0F);
}

TEST(KalmanFilterTest, SimplifiedUpdate)
{
    KalmanFilter<float, 2, 1> kf;

    Vec<float, 2> x0({0.0F, 1.0F});
    kf.reset(x0, Mat<float, 2, 2>::identity());

    Mat<float, 1, 2> H({1.0F, 0.0F});
    Mat<float, 1, 1> R({0.1F});
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);

    Vec<float, 1> z({0.5F});
    bool success = kf.update_simple(z);

    EXPECT_TRUE(success);
    EXPECT_GT(kf.get_state(0), 0.0F);
}

TEST(KalmanFilterTest, SingularInnovationCovariance)
{
    KalmanFilter<float, 2, 1> kf;

    kf.set_state(Vec<float, 2>({1.0F, 0.5F}));

    // Zero covariance and measurement matrix will cause singular S
    Mat<float, 2, 2> P = Mat<float, 2, 2>::zero();
    Mat<float, 1, 2> H({0.0F, 0.0F});  // Zero measurement matrix
    Mat<float, 1, 1> R({0.0F});        // Zero noise

    kf.set_covariance(P);
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);

    Vec<float, 1> z({2.0F});
    bool success = kf.update(z);

    // Should reject measurement due to singular covariance
    EXPECT_FALSE(success);
}

// ==================== Extended Kalman Filter Tests ====================

TEST(KalmanFilterTest, InvalidMeasurementPreservesStateAndCovariance)
{
    KalmanFilter<float, 1, 1> filter;
    filter.set_measurement_matrix(Mat<float, 1, 1>{1});
    const auto state = filter.get_state();
    const auto covariance = filter.get_covariance();
    Vec<float, 1> invalid{std::numeric_limits<float>::quiet_NaN()};
    EXPECT_FALSE(filter.update(invalid));
    EXPECT_FALSE(filter.update_simple(invalid));
    EXPECT_EQ(filter.get_state(), state);
    EXPECT_EQ(filter.get_covariance(), covariance);
    EXPECT_TRUE(std::isinf(filter.compute_nis(invalid)));
    filter.set_covariance(Mat<float, 1, 1>::zero());
    filter.set_measurement_noise(Mat<float, 1, 1>::zero());
    EXPECT_TRUE(std::isinf(filter.compute_nis(Vec<float, 1>{0})));
}

TEST(ExtendedKalmanFilterTest, ScaleAwareUpdatesAndWrappedResidual)
{
    using Filter = ExtendedKalmanFilter<float, 1, 1>;
    Filter filter;
    filter.set_measurement_model(
        +[](const Filter::StateVec& state) { return state; },
        +[](const Filter::StateVec&) { return Filter::MeasToStateMat{1}; });
    filter.set_covariance(Filter::StateMat{1e-10F});
    filter.set_measurement_noise(Filter::MeasMat{1e-10F});
    EXPECT_TRUE(filter.update(Filter::MeasVec{1}));
    const auto state = filter.get_state();
    const auto covariance = filter.get_covariance();
    Filter::MeasVec invalid{std::numeric_limits<float>::infinity()};
    EXPECT_FALSE(filter.update(invalid));
    EXPECT_FALSE(filter.update_joseph(invalid));
    EXPECT_EQ(filter.get_state(), state);
    EXPECT_EQ(filter.get_covariance(), covariance);
    filter.set_residual_function(+[](const Filter::MeasVec& measured, const Filter::MeasVec& predicted) {
        return Filter::MeasVec{std::remainder(measured[0] - predicted[0], constants::two_pi<float>())};
    });
    filter.set_state(Filter::StateVec{constants::pi<float>() - 0.01F});
    EXPECT_NEAR(filter.compute_innovation(Filter::MeasVec{-constants::pi<float>() + 0.01F})[0], 0.02F, 1e-5F);
    Filter unset;
    EXPECT_TRUE(std::isinf(unset.compute_nis(Filter::MeasVec{0})));
}

// Simple nonlinear state transition for testing
auto ekf_state_func(const Vec<float, 2>& x, float dt) -> Vec<float, 2>
{
    // x_k = [x^2, y] (nonlinear in first state)
    Vec<float, 2> x_new;
    x_new[0] = x[0] + x[1] * dt;
    x_new[1] = x[1];
    return x_new;
}

auto ekf_state_jacobian(const Vec<float, 2>& x, float dt) -> Mat<float, 2, 2>
{
    (void)x;
    return Mat<float, 2, 2>({1.0F, dt, 0.0F, 1.0F});
}

auto ekf_meas_func(const Vec<float, 2>& x) -> Vec<float, 1>
{
    // Nonlinear: z = sqrt(x^2 + y^2)
    return Vec<float, 1>({std::sqrt(x[0] * x[0] + x[1] * x[1])});
}

auto ekf_meas_jacobian(const Vec<float, 2>& x) -> Mat<float, 1, 2>
{
    float norm = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    if (norm < 1e-6F)
    {
        norm = 1e-6F;
    }
    return Mat<float, 1, 2>({x[0] / norm, x[1] / norm});
}

TEST(ExtendedKalmanFilterTest, DefaultConstruction)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;

    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 0.0F);
}

TEST(ExtendedKalmanFilterTest, ConstructionWithFunctions)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    // Should initialize successfully
    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0F);
}

TEST(ExtendedKalmanFilterTest, StateInitialization)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;

    Vec<float, 2> x0({3.0F, 4.0F});
    Mat<float, 2, 2> P0 = Mat<float, 2, 2>::identity() * 2.0F;

    ekf.reset(x0, P0);

    EXPECT_FLOAT_EQ(ekf.get_state(0), 3.0F);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 4.0F);
    EXPECT_FLOAT_EQ(ekf.get_variance(0), 2.0F);
}

TEST(ExtendedKalmanFilterTest, PredictStep)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    // Initial state
    ekf.set_state(Vec<float, 2>({1.0F, 2.0F}));

    float dt = 0.1F;
    ekf.predict(dt);

    // After prediction: x = 1.0 + 2.0*0.1 = 1.2
    EXPECT_NEAR(ekf.get_state(0), 1.2F, 1e-5F);
    EXPECT_NEAR(ekf.get_state(1), 2.0F, 1e-5F);
}

TEST(ExtendedKalmanFilterTest, UpdateStep)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    // Initial state [3, 4] has norm 5
    ekf.reset(Vec<float, 2>({3.0F, 4.0F}), Mat<float, 2, 2>::identity());

    // Measurement: norm = 6 (should pull state outward)
    Vec<float, 1> z({6.0F});
    bool success = ekf.update(z);

    EXPECT_TRUE(success);

    // State should move toward measurement
    float new_norm = std::sqrt(ekf.get_state(0) * ekf.get_state(0) + ekf.get_state(1) * ekf.get_state(1));
    EXPECT_GT(new_norm, 5.0F);
    EXPECT_LT(new_norm, 6.0F);
}

TEST(ExtendedKalmanFilterTest, UpdateJoseph)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    ekf.reset(Vec<float, 2>({3.0F, 4.0F}), Mat<float, 2, 2>::identity());

    Vec<float, 1> z({6.0F});
    bool success = ekf.update_joseph(z);

    EXPECT_TRUE(success);
}

TEST(ExtendedKalmanFilterTest, NonlinearTracking)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    ekf.reset(Vec<float, 2>({1.0F, 1.0F}), Mat<float, 2, 2>::identity());

    float dt = 0.1F;

    // Run a few prediction/update cycles
    for (int i = 0; i < 5; ++i)
    {
        ekf.predict(dt);

        // Simulated measurement
        float meas = std::sqrt(2.0F) + 0.1F * i;
        ekf.update(Vec<float, 1>({meas}));
    }

    // Should have tracked the measurements
    float final_norm = std::sqrt(ekf.get_state(0) * ekf.get_state(0) + ekf.get_state(1) * ekf.get_state(1));
    EXPECT_GT(final_norm, 1.0F);
}

TEST(ExtendedKalmanFilterTest, Innovation)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    ekf.set_state(Vec<float, 2>({3.0F, 4.0F}));  // Norm = 5

    Vec<float, 1> z({6.0F});
    Vec<float, 1> innovation = ekf.compute_innovation(z);

    EXPECT_NEAR(innovation[0], 1.0F, 1e-5F);  // 6.0 - 5.0 = 1.0
}

TEST(ExtendedKalmanFilterTest, NIS)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});

    ExtendedKalmanFilter<float, 2, 1> ekf(ekf_state_func, ekf_meas_func, ekf_state_jacobian, ekf_meas_jacobian, Q, R);

    ekf.reset(Vec<float, 2>({3.0F, 4.0F}), Mat<float, 2, 2>::identity());

    Vec<float, 1> z({6.0F});
    float nis = ekf.compute_nis(z);

    EXPECT_GT(nis, 0.0F);
}

TEST(ExtendedKalmanFilterTest, SetFunctions)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;

    ekf.set_state_transition(ekf_state_func, ekf_state_jacobian);
    ekf.set_measurement_model(ekf_meas_func, ekf_meas_jacobian);

    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01F;
    Mat<float, 1, 1> R({0.1F});
    ekf.set_process_noise(Q);
    ekf.set_measurement_noise(R);

    // Should be able to use filter now
    ekf.set_state(Vec<float, 2>({1.0F, 1.0F}));
    ekf.predict(0.1F);

    EXPECT_NEAR(ekf.get_state(0), 1.1F, 1e-5F);
}

TEST(ExtendedKalmanFilterTest, Reset)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;

    ekf.set_state(Vec<float, 2>({5.0F, 3.0F}));
    ekf.reset();

    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 0.0F);
}

TEST(ExtendedKalmanFilterTest, NullFunctionPointers)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;

    // Without function pointers set, predict should do nothing
    ekf.set_state(Vec<float, 2>({1.0F, 2.0F}));
    ekf.predict(0.1F);

    // State should remain unchanged
    EXPECT_FLOAT_EQ(ekf.get_state(0), 1.0F);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 2.0F);

    // Update should return false
    Vec<float, 1> z({5.0F});
    bool success = ekf.update(z);
    EXPECT_FALSE(success);
}

// ==================== Type Alias Tests ====================

TEST(KalmanFilterTest, TypeAliases)
{
    KalmanFilter1D<float> kf1d;
    KalmanFilter2D<float> kf2d;
    KalmanFilter4D<float> kf4d;
    KalmanFilter6D<float> kf6d;

    // Just verify they compile and initialize
    EXPECT_FLOAT_EQ(kf1d.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(kf2d.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(kf4d.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(kf6d.get_state(0), 0.0F);
}

TEST(ExtendedKalmanFilterTest, TypeAliases)
{
    EKF2D<float> ekf2d;
    EKF6D<float> ekf6d;

    EXPECT_FLOAT_EQ(ekf2d.get_state(0), 0.0F);
    EXPECT_FLOAT_EQ(ekf6d.get_state(0), 0.0F);
}
