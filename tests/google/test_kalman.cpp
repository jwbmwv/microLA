// SPDX-License-Identifier: MIT
/// @file test_kalman.cpp
/// @brief Unit tests for Kalman filter and Extended Kalman filter
/// @copyright Copyright (c) 2026 James Baldwin

#include <gtest/gtest.h>
#include <microla/kalman.hpp>
#include <microla/extended_kalman.hpp>
#include <cmath>

using namespace microla;

// ==================== Standard Kalman Filter Tests ====================

TEST(KalmanFilterTest, DefaultConstruction)
{
    KalmanFilter<float, 2, 1> kf;
    
    // State should be zero-initialized
    EXPECT_FLOAT_EQ(kf.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(kf.get_state(1), 0.0f);
    
    // Covariance should be identity
    EXPECT_FLOAT_EQ(kf.get_variance(0), 1.0f);
    EXPECT_FLOAT_EQ(kf.get_variance(1), 1.0f);
}

TEST(KalmanFilterTest, StateInitialization)
{
    KalmanFilter<float, 3, 2> kf;
    
    Vec<float, 3> x0({1.0f, 2.0f, 3.0f});
    Mat<float, 3, 3> P0 = Mat<float, 3, 3>::identity() * 2.0f;
    
    kf.reset(x0, P0);
    
    EXPECT_FLOAT_EQ(kf.get_state(0), 1.0f);
    EXPECT_FLOAT_EQ(kf.get_state(1), 2.0f);
    EXPECT_FLOAT_EQ(kf.get_state(2), 3.0f);
    EXPECT_FLOAT_EQ(kf.get_variance(0), 2.0f);
}

TEST(KalmanFilterTest, PredictStep)
{
    KalmanFilter<float, 2, 1> kf;
    
    // Set initial state
    Vec<float, 2> x0({0.0f, 1.0f});  // position=0, velocity=1
    kf.set_state(x0);
    
    // State transition: position += velocity * dt
    float dt = 0.1f;
    Mat<float, 2, 2> F({
        1.0f, dt,
        0.0f, 1.0f
    });
    kf.set_state_transition(F);
    
    // Predict
    kf.predict();
    
    // After prediction: position should be 0 + 1*0.1 = 0.1
    EXPECT_NEAR(kf.get_state(0), 0.1f, 1e-6f);
    EXPECT_NEAR(kf.get_state(1), 1.0f, 1e-6f);
}

TEST(KalmanFilterTest, UpdateStep)
{
    KalmanFilter<float, 2, 1> kf;
    
    // Initial state: position=0, velocity=1
    Vec<float, 2> x0({0.0f, 1.0f});
    Mat<float, 2, 2> P0 = Mat<float, 2, 2>::identity();
    kf.reset(x0, P0);
    
    // Measurement matrix (measure position only)
    Mat<float, 1, 2> H({1.0f, 0.0f});
    kf.set_measurement_matrix(H);
    
    // Set reasonable noise
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    kf.set_process_noise(Q);
    kf.set_measurement_noise(R);
    
    // Measurement: position = 0.5
    Vec<float, 1> z({0.5f});
    bool success = kf.update(z);
    
    EXPECT_TRUE(success);
    // State should move toward measurement
    EXPECT_GT(kf.get_state(0), 0.0f);
    EXPECT_LT(kf.get_state(0), 0.5f);
}

TEST(KalmanFilterTest, ConstantVelocityTracking)
{
    KalmanFilter<float, 2, 1> kf;
    
    float dt = 0.1f;
    Mat<float, 2, 2> F({1.0f, dt, 0.0f, 1.0f});
    Mat<float, 1, 2> H({1.0f, 0.0f});
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001f;
    Mat<float, 1, 1> R({0.1f});
    
    kf.set_state_transition(F);
    kf.set_measurement_matrix(H);
    kf.set_process_noise(Q);
    kf.set_measurement_noise(R);
    
    // Initial state
    kf.reset(Vec<float, 2>({0.0f, 1.0f}), Mat<float, 2, 2>::identity());
    
    // Simulate 10 steps with constant velocity
    for (int i = 0; i < 10; ++i)
    {
        float true_pos = 1.0f * i * dt;
        
        kf.predict();
        kf.update(Vec<float, 1>({true_pos}));
    }
    
    // Velocity estimate should converge close to 1.0
    EXPECT_NEAR(kf.get_state(1), 1.0f, 0.2f);
}

TEST(KalmanFilterTest, PredictWithControl)
{
    KalmanFilter<float, 2, 1> kf;
    
    Vec<float, 2> x0({0.0f, 0.0f});
    kf.set_state(x0);
    
    float dt = 0.1f;
    Mat<float, 2, 2> F({1.0f, dt, 0.0f, 1.0f});
    Mat<float, 2, 1> B({0.5f * dt * dt, dt});  // Control affects position and velocity
    
    kf.set_state_transition(F);
    
    // Control input: acceleration = 2.0
    Vec<float, 1> u({2.0f});
    kf.predict<1>(u, B);
    
    // Check state after control input
    // position = 0.5 * accel * dt^2 = 0.5 * 2.0 * 0.01 = 0.01
    // velocity = accel * dt = 2.0 * 0.1 = 0.2
    EXPECT_NEAR(kf.get_state(0), 0.01f, 1e-5f);
    EXPECT_NEAR(kf.get_state(1), 0.2f, 1e-5f);
}

TEST(KalmanFilterTest, Innovation)
{
    KalmanFilter<float, 2, 1> kf;
    
    kf.set_state(Vec<float, 2>({1.0f, 0.5f}));
    
    Mat<float, 1, 2> H({1.0f, 0.0f});
    kf.set_measurement_matrix(H);
    
    // Innovation should be measurement - prediction
    Vec<float, 1> z({2.0f});
    Vec<float, 1> innovation = kf.compute_innovation(z);
    
    EXPECT_NEAR(innovation[0], 1.0f, 1e-6f);  // 2.0 - 1.0 = 1.0
}

TEST(KalmanFilterTest, NIS)
{
    KalmanFilter<float, 2, 1> kf;
    
    kf.set_state(Vec<float, 2>({1.0f, 0.5f}));
    
    Mat<float, 1, 2> H({1.0f, 0.0f});
    Mat<float, 1, 1> R({0.1f});
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);
    
    // NIS should be positive for any measurement
    Vec<float, 1> z({2.0f});
    float nis = kf.compute_nis(z);
    
    EXPECT_GT(nis, 0.0f);
}

TEST(KalmanFilterTest, Reset)
{
    KalmanFilter<float, 2, 1> kf;
    
    // Set some state
    kf.set_state(Vec<float, 2>({5.0f, 3.0f}));
    
    // Reset
    kf.reset();
    
    // Should return to zero state
    EXPECT_FLOAT_EQ(kf.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(kf.get_state(1), 0.0f);
}

TEST(KalmanFilterTest, SimplifiedUpdate)
{
    KalmanFilter<float, 2, 1> kf;
    
    Vec<float, 2> x0({0.0f, 1.0f});
    kf.reset(x0, Mat<float, 2, 2>::identity());
    
    Mat<float, 1, 2> H({1.0f, 0.0f});
    Mat<float, 1, 1> R({0.1f});
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);
    
    Vec<float, 1> z({0.5f});
    bool success = kf.update_simple(z);
    
    EXPECT_TRUE(success);
    EXPECT_GT(kf.get_state(0), 0.0f);
}

TEST(KalmanFilterTest, SingularInnovationCovariance)
{
    KalmanFilter<float, 2, 1> kf;
    
    kf.set_state(Vec<float, 2>({1.0f, 0.5f}));
    
    // Zero covariance and measurement matrix will cause singular S
    Mat<float, 2, 2> P = Mat<float, 2, 2>::zero();
    Mat<float, 1, 2> H({0.0f, 0.0f});  // Zero measurement matrix
    Mat<float, 1, 1> R({0.0f});        // Zero noise
    
    kf.set_covariance(P);
    kf.set_measurement_matrix(H);
    kf.set_measurement_noise(R);
    
    Vec<float, 1> z({2.0f});
    bool success = kf.update(z);
    
    // Should reject measurement due to singular covariance
    EXPECT_FALSE(success);
}

// ==================== Extended Kalman Filter Tests ====================

// Simple nonlinear state transition for testing
Vec<float, 2> ekf_state_func(const Vec<float, 2>& x, float dt)
{
    // x_k = [x^2, y] (nonlinear in first state)
    Vec<float, 2> x_new;
    x_new[0] = x[0] + x[1] * dt;
    x_new[1] = x[1];
    return x_new;
}

Mat<float, 2, 2> ekf_state_jacobian(const Vec<float, 2>& x, float dt)
{
    (void)x;
    return Mat<float, 2, 2>({
        1.0f, dt,
        0.0f, 1.0f
    });
}

Vec<float, 1> ekf_meas_func(const Vec<float, 2>& x)
{
    // Nonlinear: z = sqrt(x^2 + y^2)
    return Vec<float, 1>({std::sqrt(x[0] * x[0] + x[1] * x[1])});
}

Mat<float, 1, 2> ekf_meas_jacobian(const Vec<float, 2>& x)
{
    float norm = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    if (norm < 1e-6f)
        norm = 1e-6f;
    return Mat<float, 1, 2>({x[0] / norm, x[1] / norm});
}

TEST(ExtendedKalmanFilterTest, DefaultConstruction)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;
    
    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 0.0f);
}

TEST(ExtendedKalmanFilterTest, ConstructionWithFunctions)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    // Should initialize successfully
    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0f);
}

TEST(ExtendedKalmanFilterTest, StateInitialization)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;
    
    Vec<float, 2> x0({3.0f, 4.0f});
    Mat<float, 2, 2> P0 = Mat<float, 2, 2>::identity() * 2.0f;
    
    ekf.reset(x0, P0);
    
    EXPECT_FLOAT_EQ(ekf.get_state(0), 3.0f);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 4.0f);
    EXPECT_FLOAT_EQ(ekf.get_variance(0), 2.0f);
}

TEST(ExtendedKalmanFilterTest, PredictStep)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    // Initial state
    ekf.set_state(Vec<float, 2>({1.0f, 2.0f}));
    
    float dt = 0.1f;
    ekf.predict(dt);
    
    // After prediction: x = 1.0 + 2.0*0.1 = 1.2
    EXPECT_NEAR(ekf.get_state(0), 1.2f, 1e-5f);
    EXPECT_NEAR(ekf.get_state(1), 2.0f, 1e-5f);
}

TEST(ExtendedKalmanFilterTest, UpdateStep)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    // Initial state [3, 4] has norm 5
    ekf.reset(Vec<float, 2>({3.0f, 4.0f}), Mat<float, 2, 2>::identity());
    
    // Measurement: norm = 6 (should pull state outward)
    Vec<float, 1> z({6.0f});
    bool success = ekf.update(z);
    
    EXPECT_TRUE(success);
    
    // State should move toward measurement
    float new_norm = std::sqrt(ekf.get_state(0) * ekf.get_state(0) + 
                                ekf.get_state(1) * ekf.get_state(1));
    EXPECT_GT(new_norm, 5.0f);
    EXPECT_LT(new_norm, 6.0f);
}

TEST(ExtendedKalmanFilterTest, UpdateJoseph)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.001f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    ekf.reset(Vec<float, 2>({3.0f, 4.0f}), Mat<float, 2, 2>::identity());
    
    Vec<float, 1> z({6.0f});
    bool success = ekf.update_joseph(z);
    
    EXPECT_TRUE(success);
}

TEST(ExtendedKalmanFilterTest, NonlinearTracking)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    ekf.reset(Vec<float, 2>({1.0f, 1.0f}), Mat<float, 2, 2>::identity());
    
    float dt = 0.1f;
    
    // Run a few prediction/update cycles
    for (int i = 0; i < 5; ++i)
    {
        ekf.predict(dt);
        
        // Simulated measurement
        float meas = std::sqrt(2.0f) + 0.1f * i;
        ekf.update(Vec<float, 1>({meas}));
    }
    
    // Should have tracked the measurements
    float final_norm = std::sqrt(ekf.get_state(0) * ekf.get_state(0) + 
                                  ekf.get_state(1) * ekf.get_state(1));
    EXPECT_GT(final_norm, 1.0f);
}

TEST(ExtendedKalmanFilterTest, Innovation)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    ekf.set_state(Vec<float, 2>({3.0f, 4.0f}));  // Norm = 5
    
    Vec<float, 1> z({6.0f});
    Vec<float, 1> innovation = ekf.compute_innovation(z);
    
    EXPECT_NEAR(innovation[0], 1.0f, 1e-5f);  // 6.0 - 5.0 = 1.0
}

TEST(ExtendedKalmanFilterTest, NIS)
{
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    
    ExtendedKalmanFilter<float, 2, 1> ekf(
        ekf_state_func,
        ekf_meas_func,
        ekf_state_jacobian,
        ekf_meas_jacobian,
        Q,
        R
    );
    
    ekf.reset(Vec<float, 2>({3.0f, 4.0f}), Mat<float, 2, 2>::identity());
    
    Vec<float, 1> z({6.0f});
    float nis = ekf.compute_nis(z);
    
    EXPECT_GT(nis, 0.0f);
}

TEST(ExtendedKalmanFilterTest, SetFunctions)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;
    
    ekf.set_state_transition(ekf_state_func, ekf_state_jacobian);
    ekf.set_measurement_model(ekf_meas_func, ekf_meas_jacobian);
    
    Mat<float, 2, 2> Q = Mat<float, 2, 2>::identity() * 0.01f;
    Mat<float, 1, 1> R({0.1f});
    ekf.set_process_noise(Q);
    ekf.set_measurement_noise(R);
    
    // Should be able to use filter now
    ekf.set_state(Vec<float, 2>({1.0f, 1.0f}));
    ekf.predict(0.1f);
    
    EXPECT_NEAR(ekf.get_state(0), 1.1f, 1e-5f);
}

TEST(ExtendedKalmanFilterTest, Reset)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;
    
    ekf.set_state(Vec<float, 2>({5.0f, 3.0f}));
    ekf.reset();
    
    EXPECT_FLOAT_EQ(ekf.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 0.0f);
}

TEST(ExtendedKalmanFilterTest, NullFunctionPointers)
{
    ExtendedKalmanFilter<float, 2, 1> ekf;
    
    // Without function pointers set, predict should do nothing
    ekf.set_state(Vec<float, 2>({1.0f, 2.0f}));
    ekf.predict(0.1f);
    
    // State should remain unchanged
    EXPECT_FLOAT_EQ(ekf.get_state(0), 1.0f);
    EXPECT_FLOAT_EQ(ekf.get_state(1), 2.0f);
    
    // Update should return false
    Vec<float, 1> z({5.0f});
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
    EXPECT_FLOAT_EQ(kf1d.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(kf2d.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(kf4d.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(kf6d.get_state(0), 0.0f);
}

TEST(ExtendedKalmanFilterTest, TypeAliases)
{
    EKF2D<float> ekf2d;
    EKF6D<float> ekf6d;
    
    EXPECT_FLOAT_EQ(ekf2d.get_state(0), 0.0f);
    EXPECT_FLOAT_EQ(ekf6d.get_state(0), 0.0f);
}
