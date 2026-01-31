// SPDX-License-Identifier: MIT
/// @file test_integration.cpp
/// @brief Integration tests combining multiple components
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <gtest/gtest.h>
#include <cmath>

#if __cplusplus >= 201703L
#include <microla/geometry.hpp>
#endif

using namespace microla;
#if __cplusplus >= 201703L
using namespace microla::geometry;
#endif
// ===== Transformation Pipeline Tests =====

TEST(IntegrationTest, FullTransformationPipeline)
{
    // Create a transformation matrix (translate + rotate + scale)
    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();

    // Scale
    Mat<float, 4, 4> scale = Mat<float, 4, 4>::identity();
    scale(0, 0) = 2.0f;
    scale(1, 1) = 2.0f;
    scale(2, 2) = 2.0f;

    // Rotation (90 degrees around Z)
    Mat<float, 3, 3> rotation = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 2.0f);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            transform(i, j) = rotation(i, j);
        }
    }

    // Translation
    transform(0, 3) = 5.0f;
    transform(1, 3) = 3.0f;
    transform(2, 3) = 1.0f;

    // Apply to point
    Vec<float, 4> point(1.0f, 0.0f, 0.0f, 1.0f);
    Vec<float, 4> transformed = transform * point;

    EXPECT_NEAR(transformed[0], 5.0f, 1e-5f);
    EXPECT_NEAR(transformed[1], 4.0f, 1e-5f);  // 1 + 3
    EXPECT_NEAR(transformed[2], 1.0f, 1e-5f);
}

TEST(IntegrationTest, QuaternionMatrixEquivalence)
{
    Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
    float angle = constants::pi<float>() / 4.0f;

    // Create rotation using quaternion
    Quaternion<float> q(axis, angle);
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rotated_q = q.rotate(v);

    // Create rotation using matrix
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(angle);
    Vec<float, 3> rotated_m = R * v;

    // Should be equivalent
    EXPECT_NEAR(rotated_q[0], rotated_m[0], 1e-5f);
    EXPECT_NEAR(rotated_q[1], rotated_m[1], 1e-5f);
    EXPECT_NEAR(rotated_q[2], rotated_m[2], 1e-5f);
}

TEST(IntegrationTest, QuaternionToMatrixConversion)
{
    Quaternion<float> q(Vec<float, 3>(1.0f, 0.0f, 0.0f), constants::pi<float>() / 3.0f);
    Mat<float, 3, 3> R = q.to_matrix();

    Vec<float, 3> v(0.0f, 1.0f, 0.0f);
    Vec<float, 3> rotated_q = q.rotate(v);
    Vec<float, 3> rotated_m = R * v;

    EXPECT_NEAR(rotated_q[0], rotated_m[0], 1e-5f);
    EXPECT_NEAR(rotated_q[1], rotated_m[1], 1e-5f);
    EXPECT_NEAR(rotated_q[2], rotated_m[2], 1e-5f);
}

#if __cplusplus >= 201703L

// ===== Geometric Transformations =====

TEST(IntegrationTest, TransformAABB)
{
    AABB<float> box(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));

    // Transform all corners
    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();
    transform(0, 3) = 5.0f;  // Translate x by 5

    Vec<float, 3> corners[8];
    corners[0] = Vec<float, 3>(-1.0f, -1.0f, -1.0f);
    corners[1] = Vec<float, 3>(1.0f, -1.0f, -1.0f);
    corners[2] = Vec<float, 3>(-1.0f, 1.0f, -1.0f);
    corners[3] = Vec<float, 3>(1.0f, 1.0f, -1.0f);
    corners[4] = Vec<float, 3>(-1.0f, -1.0f, 1.0f);
    corners[5] = Vec<float, 3>(1.0f, -1.0f, 1.0f);
    corners[6] = Vec<float, 3>(-1.0f, 1.0f, 1.0f);
    corners[7] = Vec<float, 3>(1.0f, 1.0f, 1.0f);

    // Find new bounds
    Vec<float, 4> corner_h(corners[0][0], corners[0][1], corners[0][2], 1.0f);
    Vec<float, 4> transformed_first = transform * corner_h;
    Vec<float, 3> new_min(transformed_first[0], transformed_first[1], transformed_first[2]);
    Vec<float, 3> new_max = new_min;

    for (int i = 1; i < 8; ++i)  // Start from 1 since we already processed 0
    {
        Vec<float, 4> corner_h(corners[i][0], corners[i][1], corners[i][2], 1.0f);
        Vec<float, 4> transformed = transform * corner_h;
        Vec<float, 3> transformed_3d(transformed[0], transformed[1], transformed[2]);

        new_min = new_min.min(transformed_3d);
        new_max = new_max.max(transformed_3d);
    }

    EXPECT_NEAR(new_min[0], 4.0f, 1e-5f);  // -1 + 5
    EXPECT_NEAR(new_max[0], 6.0f, 1e-5f);  // 1 + 5
}

// ===== Ray Tracing Scene =====

TEST(IntegrationTest, RayTracingScene)
{
    // Create a sphere
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 5.0f), 1.0f);

    // Cast ray from camera
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(0.0f, 0.0f, 1.0f));

    // Test intersection
    auto result = sphere.intersect(ray);  // Member function returns pair<T,T>
    EXPECT_TRUE(result.has_value());

    if (result)
    {
#if __cplusplus >= 201703L
        auto [t1, t2] = result.value();
#else
        auto t1 = std::get<0>(result.value());
#endif
        Vec<float, 3> hit_point = ray.at(t1);

        EXPECT_NEAR(hit_point[2], 4.0f, 1e-5f);  // Should hit at z=4
    }
}

TEST(IntegrationTest, RayPlaneIntersection)
{
    Plane<float> plane(Vec<float, 3>(0.0f, 1.0f, 0.0f), -2.0f);  // y=2 plane
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(0.0f, 1.0f, 0.0f));

    auto result = intersect(ray, plane);
    EXPECT_TRUE(result.has_value());

    if (result)
    {
        float t = result.value();
        Vec<float, 3> hit_point = ray.at(t);
        EXPECT_NEAR(hit_point[1], 2.0f, 1e-5f);
    }
}

// ===== Physics Simulation =====

TEST(IntegrationTest, RigidBodyRotation)
{
    // Initial orientation
    Quaternion<float> orientation = Quaternion<float>::identity();

    // Angular velocity (radians per second)
    Vec<float, 3> angular_velocity(0.0f, 0.0f, 1.0f);  // 1 rad/s around Z

    // Simulate rotation for 1 second (using small timesteps)
    float dt = 0.01f;
    int steps = 100;

    for (int i = 0; i < steps; ++i)
    {
        float angle = angular_velocity.length() * dt;
        Vec<float, 3> axis = angular_velocity.normalized();
        Quaternion<float> delta_q(axis, angle);
        orientation = delta_q * orientation;
        orientation.normalize();
    }

    // After 1 second, should have rotated by 1 radian
    Vec<float, 3> euler = orientation.to_euler();
    EXPECT_NEAR(euler[2], 1.0f, 0.01f);
}

// ===== Camera System =====

TEST(IntegrationTest, CameraLookAt)
{
    Vec<float, 3> camera_pos(0.0f, 0.0f, 0.0f);
    Vec<float, 3> target(0.0f, 0.0f, -1.0f);
    Vec<float, 3> up(0.0f, 1.0f, 0.0f);

    // Compute forward direction
    Vec<float, 3> forward = (target - camera_pos).normalized();

    // Create look-at matrix
    Mat<float, 3, 3> view_rotation = Mat<float, 3, 3>::look_at(forward, up);

    // Transform a point in front of camera
    Vec<float, 3> world_point(1.0f, 0.0f, -5.0f);
    Vec<float, 3> view_point = view_rotation * world_point;

    // Should be to the right in view space
    EXPECT_GT(view_point[0], 0.0f);
}

// ===== Collision Detection =====

TEST(IntegrationTest, SphereAABBCollision)
{
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 1.0f);
    AABB<float> box(Vec<float, 3>(-2.0f, -0.5f, -0.5f), Vec<float, 3>(2.0f, 0.5f, 0.5f));

    EXPECT_TRUE(sphere.intersects(box));
}

TEST(IntegrationTest, FrustumCulling)
{
    // Create view-projection matrix
    Mat<float, 4, 4> vp = Mat<float, 4, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);

    // Test objects
    Sphere<float> visible_sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 0.5f);
    EXPECT_TRUE(frustum.contains(visible_sphere));
}

#endif  // C++17

// ===== Linear Algebra Operations =====

TEST(IntegrationTest, SolveLinearSystem)
{
    // Solve Ax = b
    Mat<float, 3, 3> A({2.0f, -1.0f, 0.0f, -1.0f, 2.0f, -1.0f, 0.0f, -1.0f, 2.0f});
    Vec<float, 3> b(1.0f, 0.0f, 1.0f);

    // Using LU decomposition
#if __cplusplus >= 201703L
    auto [L, U, P] = A.lu();
#else
    auto lu_result = A.lu();
    auto L = std::get<0>(lu_result);
    auto U = std::get<1>(lu_result);
    auto P = std::get<2>(lu_result);
#endif

    // Check decomposition is correct
    Mat<float, 3, 3> reconstructed = P * L * U;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), A(i, j), 1e-5f);
        }
    }
}

TEST(IntegrationTest, EigenvalueApplication)
{
    // Symmetric matrix
    Mat<float, 3, 3> A({2.0f, -1.0f, 0.0f, -1.0f, 2.0f, -1.0f, 0.0f, -1.0f, 2.0f});

    std::vector<float> eigenvalues = A.eigenvaluesQR();

    // Should have 3 real eigenvalues
    EXPECT_EQ(eigenvalues.size(), 3);

    // All eigenvalues should be positive (positive definite matrix)
    for (float ev : eigenvalues)
    {
        EXPECT_GT(ev, 0.0f);
    }
}

// ===== Interpolation =====

TEST(IntegrationTest, CompleteInterpolationPipeline)
{
    // Vector interpolation
    Vec<float, 3> v1(0.0f, 0.0f, 0.0f);
    Vec<float, 3> v2(1.0f, 1.0f, 1.0f);
    Vec<float, 3> v_mid = v1.lerp(v2, 0.5f);

    EXPECT_NEAR(v_mid[0], 0.5f, 1e-6f);

    // Quaternion interpolation
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2(Vec<float, 3>(0.0f, 0.0f, 1.0f), constants::pi<float>() / 2.0f);
    Quaternion<float> q_mid = q1.slerp(q2, 0.5f);

    // Should be halfway between rotations
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rotated = q_mid.rotate(v);

    // Should be at 45 degrees
    EXPECT_NEAR(rotated[0], rotated[1], 0.01f);
}

// ===== Matrix Chain Multiplication =====

TEST(IntegrationTest, MatrixChainMultiplication)
{
    Mat<float, 2, 3> A({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    Mat<float, 3, 2> B({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    Mat<float, 2, 2> C({1.0f, 0.0f, 0.0f, 1.0f});

    Mat<float, 2, 2> result = (A * B) * C;

    EXPECT_FLOAT_EQ(result(0, 0), 22.0f);
    EXPECT_FLOAT_EQ(result(0, 1), 28.0f);
}

// ===== Numerical Stability =====

TEST(IntegrationTest, NumericalStabilityLargeRotations)
{
    Quaternion<float> q = Quaternion<float>::identity();

    // Apply many small rotations
    Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
    float small_angle = 0.001f;

    for (int i = 0; i < 1000; ++i)
    {
        Quaternion<float> delta(axis, small_angle);
        q = delta * q;
        q.normalize();
    }

    // Should still be unit quaternion
    EXPECT_NEAR(q.norm(), 1.0f, 1e-5f);

    // Total rotation should be approximately 1 radian
    Vec<float, 3> euler = q.to_euler();
    EXPECT_NEAR(euler[2], 1.0f, 0.01f);
}

// ===== Performance Optimization Tests =====

TEST(IntegrationTest, MatrixVectorBatchOperations)
{
    Mat<float, 3, 3> transform = Mat<float, 3, 3>::rotation_z(0.1f);

    std::vector<Vec<float, 3>> points;
    for (int i = 0; i < 100; ++i)
    {
        points.push_back(Vec<float, 3>(static_cast<float>(i), 0.0f, 0.0f));
    }

    std::vector<Vec<float, 3>> transformed;
    for (const auto& p : points)
    {
        transformed.push_back(transform * p);
    }

    EXPECT_EQ(transformed.size(), 100);
    EXPECT_TRUE(std::isfinite(transformed[50][0]));
}
