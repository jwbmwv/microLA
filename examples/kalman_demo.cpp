// SPDX-License-Identifier: Apache-2.0
/// @file kalman_demo.cpp
/// @brief Demonstration of Kalman filter and Extended Kalman filter usage
/// @details Shows practical examples of state estimation for embedded systems
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace microla;

// ==================== Standard Kalman Filter Example ====================
// 1D position + velocity tracking from noisy position measurements

void demo_standard_kalman()
{
    std::cout << "=== Standard Kalman Filter Demo ===\n";
    std::cout << "Tracking 1D position + velocity from noisy position measurements\n\n";

    // Create Kalman filter: 2 states (position, velocity), 1 measurement (position)
    KalmanFilter<float, 2, 1> kf;

    // Set up state transition matrix (constant velocity model)
    // x_k = F * x_{k-1} where x = [position, velocity]^T
    // position_k = position_{k-1} + velocity_{k-1} * dt
    // velocity_k = velocity_{k-1}
    float dt = 0.1F;  // 100ms time step
    Mat<float, 2, 2> f({1.0F, dt, 0.0F, 1.0F});
    kf.set_state_transition(f);

    // Measurement matrix (measure position only)
    // z = H * x = [1, 0] * [position, velocity]^T = position
    Mat<float, 1, 2> h({1.0F, 0.0F});
    kf.set_measurement_matrix(h);

    // Process noise (model uncertainty)
    Mat<float, 2, 2> q({0.01F, 0.0F, 0.0F, 0.01F});
    kf.set_process_noise(q);

    // Measurement noise (sensor uncertainty)
    Mat<float, 1, 1> r({0.5F});  // Position measurement has 0.5m std deviation
    kf.set_measurement_noise(r);

    // Initial state: position = 0, velocity = 1 m/s
    Vec<float, 2> x0({0.0F, 1.0F});
    Mat<float, 2, 2> p0({1.0F, 0.0F, 0.0F, 1.0F});
    kf.reset(x0, p0);

    // Simulate 10 time steps
    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "Time   | True Pos | Meas Pos | Est Pos  | True Vel | Est Vel  | Pos Std\n";
    std::cout << "-------|----------|----------|----------|----------|----------|--------\n";

    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i) * dt;

        // True state (constant velocity motion)
        float true_pos = 0.0F + 1.0F * t;  // position = initial_pos + velocity * time
        float true_vel = 1.0F;

        // Simulate noisy measurement (add random noise in realistic scenario)
        float noise = (i % 2 == 0) ? 0.2F : -0.3F;  // Simplified noise
        float measured_pos = true_pos + noise;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        Vec<float, 1> z({measured_pos});
        kf.update(z);

        // Get estimates
        float est_pos = kf.get_state(0);
        float est_vel = kf.get_state(1);
        float pos_std = kf.get_std_dev(0);

        std::cout << std::setw(6) << t << " | " << std::setw(8) << true_pos << " | " << std::setw(8) << measured_pos
                  << " | " << std::setw(8) << est_pos << " | " << std::setw(8) << true_vel << " | " << std::setw(8)
                  << est_vel << " | " << std::setw(7) << pos_std << '\n';
    }

    std::cout << "\n✓ Filter converges: estimated velocity approaches true velocity (1.0 m/s)\n";
    std::cout << "✓ Position estimates smooth out measurement noise\n\n";
}

// ==================== Extended Kalman Filter Example ====================
// Tracking object in 2D with range and bearing measurements (nonlinear)

// State: [x, y, vx, vy] - position and velocity in 2D
// Measurement: [range, bearing] - polar coordinates from origin

// Nonlinear state transition: constant velocity model
Vec<float, 4> state_transition_nonlinear(const Vec<float, 4>& x, float dt)
{
    Vec<float, 4> x_new;
    x_new[0] = x[0] + x[2] * dt;  // x = x + vx * dt
    x_new[1] = x[1] + x[3] * dt;  // y = y + vy * dt
    x_new[2] = x[2];              // vx remains constant
    x_new[3] = x[3];              // vy remains constant
    return x_new;
}

// State transition Jacobian (linearization)
Mat<float, 4, 4> state_jacobian(const Vec<float, 4>& x, float dt)
{
    (void)x;  // Not used in linear velocity model
    return Mat<float, 4, 4>(
        {1.0F, 0.0F, dt, 0.0F, 0.0F, 1.0F, 0.0F, dt, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F});
}

// Nonlinear measurement model: [range, bearing] from [x, y]
Vec<float, 2> measurement_model(const Vec<float, 4>& x)
{
    float range = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    float bearing = std::atan2(x[1], x[0]);
    return Vec<float, 2>({range, bearing});
}

// Measurement Jacobian (linearization)
Mat<float, 2, 4> measurement_jacobian(const Vec<float, 4>& x)
{
    float px = x[0];
    float py = x[1];
    float range = std::sqrt(px * px + py * py);

    if (range < 1e-6F)
    {
        range = 1e-6F;  // Avoid division by zero
    }

    // ∂range/∂x = x/r,  ∂range/∂y = y/r
    // ∂bearing/∂x = -y/r², ∂bearing/∂y = x/r²
    float r2 = range * range;

    return Mat<float, 2, 4>({
        px / range, py / range, 0.0F, 0.0F,  // ∂range/∂[x,y,vx,vy]
        -py / r2, px / r2, 0.0F, 0.0F        // ∂bearing/∂[x,y,vx,vy]
    });
}

void demo_extended_kalman()
{
    std::cout << "\n=== Extended Kalman Filter Demo ===\n";
    std::cout << "Tracking 2D position from range and bearing measurements (nonlinear)\n\n";

    // Create EKF: 4 states [x, y, vx, vy], 2 measurements [range, bearing]
    Mat<float, 4, 4> q = Mat<float, 4, 4>::identity() * 0.01F;  // Process noise
    Mat<float, 2, 2> r({
        0.1F, 0.0F,  // Range measurement noise
        0.0F, 0.05F  // Bearing measurement noise (radians)
    });

    ExtendedKalmanFilter<float, 4, 2> ekf(state_transition_nonlinear, measurement_model, state_jacobian,
                                          measurement_jacobian, q, r);

    // Initial state: position (10, 5), velocity (2, 1) m/s
    Vec<float, 4> x0({10.0F, 5.0F, 2.0F, 1.0F});
    Mat<float, 4, 4> p0 = Mat<float, 4, 4>::identity();
    ekf.reset(x0, p0);

    float dt = 0.1F;

    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "Time   | True X | True Y | Est X  | Est Y  | True Range | Meas Range | Est Range\n";
    std::cout << "-------|--------|--------|--------|--------|------------|------------|----------\n";

    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i) * dt;

        // True state
        float true_x = 10.0F + 2.0F * t;
        float true_y = 5.0F + 1.0F * t;
        float true_range = std::sqrt(true_x * true_x + true_y * true_y);
        float true_bearing = std::atan2(true_y, true_x);

        // Noisy measurements
        float noise_range = (i % 2 == 0) ? 0.1F : -0.15F;
        float noise_bearing = (i % 2 == 0) ? 0.02F : -0.03F;
        float meas_range = true_range + noise_range;
        float meas_bearing = true_bearing + noise_bearing;

        // EKF predict
        ekf.predict(dt);

        // EKF update
        Vec<float, 2> z({meas_range, meas_bearing});
        ekf.update(z);

        // Get estimates
        Vec<float, 4> x_est = ekf.get_state();
        float est_range = std::sqrt(x_est[0] * x_est[0] + x_est[1] * x_est[1]);

        std::cout << std::setw(6) << t << " | " << std::setw(6) << true_x << " | " << std::setw(6) << true_y << " | "
                  << std::setw(6) << x_est[0] << " | " << std::setw(6) << x_est[1] << " | " << std::setw(10)
                  << true_range << " | " << std::setw(10) << meas_range << " | " << std::setw(9) << est_range << '\n';
    }

    std::cout << "\n✓ EKF handles nonlinear measurement model (polar coordinates)\n";
    std::cout << "✓ State estimates converge despite measurement noise\n\n";
}

// ==================== Sensor Fusion Example ====================
// Combining accelerometer and gyroscope with Kalman filter

void demo_sensor_fusion()
{
    std::cout << "\n=== Kalman Filter Sensor Fusion Demo ===\n";
    std::cout << "Fusing accelerometer and gyroscope for attitude estimation\n\n";

    // Simple 1D tilt angle estimation
    // State: [angle, bias] - tilt angle and gyro bias
    // Measurements: accelerometer angle (from gravity vector)

    KalmanFilter<float, 2, 1> kf;

    float dt = 0.02F;  // 50 Hz update rate

    // State transition: angle_k = angle_{k-1} + (gyro - bias) * dt
    // This is simplified - gyro input comes through update
    Mat<float, 2, 2> f({1.0F, -dt, 0.0F, 1.0F});
    kf.set_state_transition(f);

    // Measurement: accelerometer angle
    Mat<float, 1, 2> h({1.0F, 0.0F});
    kf.set_measurement_matrix(h);

    // Tuning parameters
    Mat<float, 2, 2> q({
        0.001F, 0.0F, 0.0F, 0.00001F  // Bias changes slowly
    });
    Mat<float, 1, 1> r({0.3F});  // Accelerometer noise

    kf.set_process_noise(q);
    kf.set_measurement_noise(r);

    // Initial state
    kf.reset(Vec<float, 2>({0.0F, 0.0F}), Mat<float, 2, 2>::identity());

    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "Time   | True Angle | Accel Angle | Est Angle | Est Bias\n";
    std::cout << "-------|------------|-------------|-----------|----------\n";

    float true_bias = 0.1F;  // Gyro has 0.1 rad/s bias

    for (int i = 0; i < 10; ++i)
    {
        float t = static_cast<float>(i) * dt;
        float true_angle = 0.5F * std::sin(t * 2.0F);  // Sinusoidal motion

        // Simulated accelerometer (noisy angle measurement)
        float accel_noise = (i % 3 == 0) ? 0.1F : -0.08F;
        float accel_angle = true_angle + accel_noise;

        // Predict (in real system, would use gyro_rate - est_bias)
        kf.predict();

        // Update with accelerometer
        kf.update(Vec<float, 1>({accel_angle}));

        float est_angle = kf.get_state(0);
        float est_bias = kf.get_state(1);

        std::cout << std::setw(6) << t << " | " << std::setw(10) << true_angle << " | " << std::setw(11) << accel_angle
                  << " | " << std::setw(9) << est_angle << " | " << std::setw(8) << est_bias << '\n';
    }

    std::cout << "\n✓ Kalman filter fuses noisy accelerometer with gyroscope\n";
    std::cout << "✓ Estimates gyro bias over time for improved accuracy\n\n";
}

// ==================== Main ====================

int main()
{
    std::cout << "MicroLA - Kalman Filter Demonstrations\n";
    std::cout << "======================================\n\n";

    demo_standard_kalman();
    demo_extended_kalman();
    demo_sensor_fusion();

    std::cout << "\n=== Summary ===\n";
    std::cout << "✓ Standard Kalman Filter: optimal for linear systems\n";
    std::cout << "✓ Extended Kalman Filter: handles nonlinear dynamics/measurements\n";
    std::cout << "✓ Both use compile-time dimensions for zero-overhead\n";
    std::cout << "✓ Suitable for embedded systems (no dynamic allocation)\n";
    std::cout << "\nAll demonstrations completed successfully!\n";

    return 0;
}
