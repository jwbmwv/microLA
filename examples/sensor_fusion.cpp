// SPDX-License-Identifier: MIT
/// @file sensor_fusion.cpp
/// @brief IMU sensor fusion example using MicroLA
/// @details Demonstrates quaternion-based attitude estimation from accelerometer and gyroscope data
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace microla;

// Type aliases for convenience
using Vec3f = Vec<float, 3>;
using Mat3f = Mat<float, 3, 3>;

/// @brief Simple complementary filter for IMU sensor fusion
/// @details Fuses gyroscope (fast, drifts) and accelerometer (slow, noisy) data
class ComplementaryFilter
{
private:
    Quaternion<float> orientation_;
    float alpha_;  // Filter coefficient (0.9-0.98 typical)

public:
    ComplementaryFilter(float alpha = 0.98f) : orientation_(Quaternion<float>::identity()), alpha_(alpha) {}

    /// @brief Update orientation with new IMU readings
    /// @param gyro Gyroscope reading (rad/s) in body frame
    /// @param accel Accelerometer reading (m/s^2) in body frame
    /// @param dt Time step (seconds)
    void update(const Vec3f& gyro, const Vec3f& accel, float dt)
    {
        // 1. Integrate gyroscope (prediction step)
        Quaternion<float> gyro_quat = Quaternion<float>::from_axis_angle(gyro, gyro.length() * dt);
        Quaternion<float> predicted = orientation_ * gyro_quat;
        predicted = predicted.normalized();

        // 2. Get orientation from accelerometer (correction step)
        // Assumes accelerometer measures gravity when stationary
        Vec3f accel_norm = accel.normalized();
        Vec3f gravity{0.0f, 0.0f, -1.0f};  // Z-up convention

        // Compute rotation quaternion from accel to gravity
        Vec3f axis = accel_norm.cross(gravity);
        float angle = std::acos(clamp(accel_norm.dot(gravity), -1.0f, 1.0f));

        Quaternion<float> accel_quat;
        if (axis.length_squared() > 1e-6f)
        {
            accel_quat = Quaternion<float>::from_axis_angle(axis, angle);
        }
        else
        {
            accel_quat = Quaternion<float>::identity();
        }

        // 3. Complementary filter fusion
        orientation_ = predicted.slerp(accel_quat, 1.0f - alpha_);
        orientation_ = orientation_.normalized();
    }

    /// @brief Get current orientation quaternion
    Quaternion<float> get_orientation() const { return orientation_; }

    /// @brief Get Euler angles (roll, pitch, yaw) in radians
    Vec3f get_euler_angles() const { return orientation_.to_euler(); }

    /// @brief Get rotation matrix representation
    SquareMat<float, 3> get_rotation_matrix() const { return orientation_.to_matrix(); }
};

/// @brief Simulate IMU sensor readings
struct IMUSensor
{
    Vec3f gyro;   // Angular velocity (rad/s)
    Vec3f accel;  // Acceleration (m/s^2)
};

/// @brief Generate simulated IMU data for a rotating object
IMUSensor simulate_imu(float time)
{
    IMUSensor imu;

    // Simulate rotation around Z-axis at 30 deg/s
    const float rotation_rate = deg_to_rad(30.0f);
    imu.gyro = Vec3f{0.0f, 0.0f, rotation_rate};

    // Simulate gravity with some noise
    const float noise = 0.1f * std::sin(time * 10.0f);
    imu.accel = Vec3f{noise, noise, -9.81f};

    return imu;
}

int main()
{
    std::cout << "MicroLA - IMU Sensor Fusion Example\n";
    std::cout << "======================================\n\n";

    ComplementaryFilter filter(0.98f);

    const float dt = 0.01f;       // 100 Hz update rate
    const float duration = 5.0f;  // 5 seconds

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s)  Roll(°)  Pitch(°)  Yaw(°)\n";
    std::cout << "-------------------------------------\n";

    for (float t = 0.0f; t <= duration; t += dt)
    {
        // Get simulated IMU readings
        IMUSensor imu = simulate_imu(t);

        // Update filter
        filter.update(imu.gyro, imu.accel, dt);

        // Print orientation every 0.5 seconds
        if (std::fmod(t, 0.5f) < dt)
        {
            Vec3f euler = filter.get_euler_angles();
            std::cout << std::setw(6) << t << "   " << std::setw(6) << rad_to_deg(euler[0]) << "   " << std::setw(6)
                      << rad_to_deg(euler[1]) << "   " << std::setw(6) << rad_to_deg(euler[2]) << "\n";
        }
    }

    // Final orientation
    std::cout << "\n=== Final Orientation ===\n";
    Quaternion<float> final_quat = filter.get_orientation();
    std::cout << "Quaternion: [w=" << final_quat[Quaternion<float>::W] << ", x=" << final_quat[Quaternion<float>::X]
              << ", y=" << final_quat[Quaternion<float>::Y] << ", z=" << final_quat[Quaternion<float>::Z] << "]\n";

    SquareMat<float, 3> rot_matrix = filter.get_rotation_matrix();

    // LU decomposition
#if __cplusplus >= 201703L
    auto [L, U, P] = rot_matrix.lu();
#else
    auto lu_result = rot_matrix.lu();
    auto L = std::get<0>(lu_result);
    auto U = std::get<1>(lu_result);
    auto P = std::get<2>(lu_result);
#endif
    std::cout << "LU Decomposition:\n";
    std::cout << "L:\n";
    for (uint32_t i = 0; i < 3; ++i)
    {
        std::cout << "  [";
        for (uint32_t j = 0; j < 3; ++j)
        {
            std::cout << std::setw(8) << L(i, j);
        }
        std::cout << " ]\n";
    }

    // Eigenvalues
    auto eigenvalues = rot_matrix.eigenvaluesQR();
    std::cout << "Eigenvalues: [";
    for (uint32_t i = 0; i < 3; ++i)
    {
        std::cout << eigenvalues[i] << (i < 2 ? ", " : "");
    }
    std::cout << "]\n";

    // Demonstrate coordinate transformation
    std::cout << "\n=== Coordinate Transformation ===\n";
    Vec3f body_vector{1.0f, 0.0f, 0.0f};  // X-axis in body frame
    Vec3f world_vector = rot_matrix * body_vector;
    std::cout << "Body frame: [" << body_vector[0] << ", " << body_vector[1] << ", " << body_vector[2] << "]\n";
    std::cout << "World frame: [" << world_vector[0] << ", " << world_vector[1] << ", " << world_vector[2] << "]\n";

    return 0;
}
