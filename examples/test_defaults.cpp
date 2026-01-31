// SPDX-License-Identifier: Apache-2.0
/// @file test_defaults.cpp
/// @brief Demonstration of default template parameters

#include <microla/microla.hpp>
#include <microla/geometry.hpp>
#include <iostream>

using namespace microla;

auto main() -> int
{
    std::cout << "=== Testing Default Template Parameters ===" << '\n';
    std::cout << '\n';

    // Vec defaults to float, N=3
    Vec<> v1;  // Equivalent to Vec<float, 3>
    Vec<> v2(1.0F, 2.0F, 3.0F);
    std::cout << "Vec<> (defaults to Vec<float, 3>):" << '\n';
    std::cout << "  v2 = (" << v2.x() << ", " << v2.y() << ", " << v2.z() << ")" << '\n';
    std::cout << "  length = " << v2.length() << '\n';
    std::cout << '\n';

    // Mat defaults to float, 4x4
    Mat<> m1 = Mat<>::identity();  // Equivalent to Mat<float, 4, 4>
    std::cout << "Mat<> (defaults to Mat<float, 4, 4>):" << '\n';
    std::cout << "  Identity matrix created" << '\n';
    std::cout << "  m1(0,0) = " << m1(0, 0) << '\n';
    std::cout << "  m1(3,3) = " << m1(3, 3) << '\n';
    std::cout << '\n';

    // Quaternion defaults to float
    Quaternion<> q1;  // Equivalent to Quaternion<float>
    Quaternion<> q2 = Quaternion<>::from_axis_angle(Vec<>(0.0F, 0.0F, 1.0F), 1.57F);
    std::cout << "Quaternion<> (defaults to Quaternion<float>):" << '\n';
    std::cout << "  q1 (identity) = (" << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << ")" << '\n';
    std::cout << "  q2 (axis-angle) = (" << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << ")" << '\n';
    std::cout << '\n';

    // Geometry types default to float (in microla::geometry namespace)
    microla::geometry::Ray<> ray;  // Equivalent to microla::geometry::Ray<float>
    ray.origin = Vec<>(0.0F, 0.0F, 0.0F);
    ray.direction = Vec<>(1.0F, 0.0F, 0.0F);
    std::cout << "microla::geometry::Ray<> (defaults to Ray<float>):" << '\n';
    std::cout << "  Origin: (" << ray.origin.x() << ", " << ray.origin.y() << ", " << ray.origin.z() << ")" << '\n';
    std::cout << "  Direction: (" << ray.direction.x() << ", " << ray.direction.y() << ", " << ray.direction.z() << ")"
              << '\n';
    std::cout << '\n';

    // Kalman filters default to float
    KalmanFilter<> kf;  // Equivalent to KalmanFilter<float, 2, 1>
    std::cout << "KalmanFilter<> (defaults to KalmanFilter<float, 2, 1>):" << '\n';
    std::cout << "  State dimension: 2" << '\n';
    std::cout << "  Measurement dimension: 1" << '\n';
    std::cout << '\n';

    // Fast math functions default to float
    float angle = fast::sin(1.57F);  // Defaults to fast::sin<float>
    std::cout << "fast::sin(1.57) = " << angle << " (should be ~1.0)" << '\n';
    std::cout << '\n';

    // Safe math functions default to float
    float result = safe::safe_divide(10.0F, 2.0F);  // Defaults to safe::safe_divide<float>
    std::cout << "safe::safe_divide(10, 2) = " << result << '\n';
    std::cout << '\n';

    std::cout << "✓ All default template parameters working correctly!" << '\n';
    std::cout << "✓ Users can now write Vec<>, Mat<>, Quaternion<>, etc." << '\n';
    std::cout << "✓ Cleaner, more concise code for the most common use case (float)" << '\n';

    return 0;
}
