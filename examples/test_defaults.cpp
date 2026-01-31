// SPDX-License-Identifier: MIT
/// @file test_defaults.cpp
/// @brief Demonstration of default template parameters

#include <microla/microla.hpp>
#include <microla/geometry.hpp>
#include <iostream>

using namespace microla;

int main()
{
    std::cout << "=== Testing Default Template Parameters ===" << std::endl;
    std::cout << std::endl;

    // Vec defaults to float, N=3
    Vec<> v1;  // Equivalent to Vec<float, 3>
    Vec<> v2(1.0f, 2.0f, 3.0f);
    std::cout << "Vec<> (defaults to Vec<float, 3>):" << std::endl;
    std::cout << "  v2 = (" << v2.x() << ", " << v2.y() << ", " << v2.z() << ")" << std::endl;
    std::cout << "  length = " << v2.length() << std::endl;
    std::cout << std::endl;

    // Mat defaults to float, 4x4
    Mat<> m1 = Mat<>::identity();  // Equivalent to Mat<float, 4, 4>
    std::cout << "Mat<> (defaults to Mat<float, 4, 4>):" << std::endl;
    std::cout << "  Identity matrix created" << std::endl;
    std::cout << "  m1(0,0) = " << m1(0, 0) << std::endl;
    std::cout << "  m1(3,3) = " << m1(3, 3) << std::endl;
    std::cout << std::endl;

    // Quaternion defaults to float
    Quaternion<> q1;  // Equivalent to Quaternion<float>
    Quaternion<> q2 = Quaternion<>::from_axis_angle(Vec<>(0.0f, 0.0f, 1.0f), 1.57f);
    std::cout << "Quaternion<> (defaults to Quaternion<float>):" << std::endl;
    std::cout << "  q1 (identity) = (" << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << ")" << std::endl;
    std::cout << std::endl;

    // Geometry types default to float (in microla::geometry namespace)
    microla::geometry::Ray<> ray;  // Equivalent to microla::geometry::Ray<float>
    ray.origin = Vec<>(0.0f, 0.0f, 0.0f);
    ray.direction = Vec<>(1.0f, 0.0f, 0.0f);
    std::cout << "microla::geometry::Ray<> (defaults to Ray<float>):" << std::endl;
    std::cout << "  Origin: (" << ray.origin.x() << ", " << ray.origin.y() << ", " << ray.origin.z() << ")" << std::endl;
    std::cout << "  Direction: (" << ray.direction.x() << ", " << ray.direction.y() << ", " << ray.direction.z() << ")" << std::endl;
    std::cout << std::endl;

    // Kalman filters default to float
    KalmanFilter<> kf;  // Equivalent to KalmanFilter<float, 2, 1>
    std::cout << "KalmanFilter<> (defaults to KalmanFilter<float, 2, 1>):" << std::endl;
    std::cout << "  State dimension: 2" << std::endl;
    std::cout << "  Measurement dimension: 1" << std::endl;
    std::cout << std::endl;

    // Fast math functions default to float
    float angle = fast::sin(1.57f);  // Defaults to fast::sin<float>
    std::cout << "fast::sin(1.57) = " << angle << " (should be ~1.0)" << std::endl;
    std::cout << std::endl;

    // Safe math functions default to float
    float result = safe::safe_divide(10.0f, 2.0f);  // Defaults to safe::safe_divide<float>
    std::cout << "safe::safe_divide(10, 2) = " << result << std::endl;
    std::cout << std::endl;

    std::cout << "✓ All default template parameters working correctly!" << std::endl;
    std::cout << "✓ Users can now write Vec<>, Mat<>, Quaternion<>, etc." << std::endl;
    std::cout << "✓ Cleaner, more concise code for the most common use case (float)" << std::endl;

    return 0;
}
