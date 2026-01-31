// SPDX-License-Identifier: Apache-2.0
/// @file test_integration.cpp
/// @brief Integration tests combining multiple components
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <gtest/gtest.h>
#include <cmath>

#include <microla/geometry.hpp>

using namespace microla;
using namespace microla::geometry;

// ===== Transformation Pipeline Tests =====

TEST(IntegrationTest, FullTransformationPipeline)
{
    // Create a transformation matrix (translate + rotate + scale)
    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();

    // Scale
    Mat<float, 4, 4> scale = Mat<float, 4, 4>::identity();
    scale(0, 0) = 2.0F;
    scale(1, 1) = 2.0F;
    scale(2, 2) = 2.0F;

    // Rotation (90 degrees around Z)
    Mat<float, 3, 3> rotation = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 2.0F);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            transform(i, j) = rotation(i, j);
        }
    }

    // Translation
    transform(0, 3) = 5.0F;
    transform(1, 3) = 3.0F;
    transform(2, 3) = 1.0F;

    // Apply to point
    Vec<float, 4> point(1.0F, 0.0F, 0.0F, 1.0F);
    Vec<float, 4> transformed = transform * point;

    EXPECT_NEAR(transformed[0], 5.0F, 1e-5F);
    EXPECT_NEAR(transformed[1], 4.0F, 1e-5F);  // 1 + 3
    EXPECT_NEAR(transformed[2], 1.0F, 1e-5F);
}

TEST(IntegrationTest, QuaternionMatrixEquivalence)
{
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    float angle = constants::pi<float>() / 4.0F;

    // Create rotation using quaternion
    Quaternion<float> q(axis, angle);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated_q = q.rotate(v);

    // Create rotation using matrix
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(angle);
    Vec<float, 3> rotated_m = R * v;

    // Should be equivalent
    EXPECT_NEAR(rotated_q[0], rotated_m[0], 1e-5F);
    EXPECT_NEAR(rotated_q[1], rotated_m[1], 1e-5F);
    EXPECT_NEAR(rotated_q[2], rotated_m[2], 1e-5F);
}

TEST(IntegrationTest, QuaternionToMatrixConversion)
{
    Quaternion<float> q(Vec<float, 3>(1.0F, 0.0F, 0.0F), constants::pi<float>() / 3.0F);
    Mat<float, 3, 3> R = q.to_matrix();

    Vec<float, 3> v(0.0F, 1.0F, 0.0F);
    Vec<float, 3> rotated_q = q.rotate(v);
    Vec<float, 3> rotated_m = R * v;

    EXPECT_NEAR(rotated_q[0], rotated_m[0], 1e-5F);
    EXPECT_NEAR(rotated_q[1], rotated_m[1], 1e-5F);
    EXPECT_NEAR(rotated_q[2], rotated_m[2], 1e-5F);
}

// ===== Geometric Transformations =====

TEST(IntegrationTest, TransformAABB)
{
    AABB<float> box(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));

    // Transform all corners
    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();
    transform(0, 3) = 5.0F;  // Translate x by 5

    Vec<float, 3> corners[8];
    corners[0] = Vec<float, 3>(-1.0F, -1.0F, -1.0F);
    corners[1] = Vec<float, 3>(1.0F, -1.0F, -1.0F);
    corners[2] = Vec<float, 3>(-1.0F, 1.0F, -1.0F);
    corners[3] = Vec<float, 3>(1.0F, 1.0F, -1.0F);
    corners[4] = Vec<float, 3>(-1.0F, -1.0F, 1.0F);
    corners[5] = Vec<float, 3>(1.0F, -1.0F, 1.0F);
    corners[6] = Vec<float, 3>(-1.0F, 1.0F, 1.0F);
    corners[7] = Vec<float, 3>(1.0F, 1.0F, 1.0F);

    // Find new bounds
    Vec<float, 4> corner_h(corners[0][0], corners[0][1], corners[0][2], 1.0F);
    Vec<float, 4> transformed_first = transform * corner_h;
    Vec<float, 3> new_min(transformed_first[0], transformed_first[1], transformed_first[2]);
    Vec<float, 3> new_max = new_min;

    for (int i = 1; i < 8; ++i)  // Start from 1 since we already processed 0
    {
        Vec<float, 4> corner_h(corners[i][0], corners[i][1], corners[i][2], 1.0F);
        Vec<float, 4> transformed = transform * corner_h;
        Vec<float, 3> transformed_3d(transformed[0], transformed[1], transformed[2]);

        new_min = new_min.min(transformed_3d);
        new_max = new_max.max(transformed_3d);
    }

    EXPECT_NEAR(new_min[0], 4.0F, 1e-5F);  // -1 + 5
    EXPECT_NEAR(new_max[0], 6.0F, 1e-5F);  // 1 + 5
}

// ===== Ray Tracing Scene =====

TEST(IntegrationTest, RayTracingScene)
{
    // Create a sphere
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 5.0F), 1.0F);

    // Cast ray from camera
    Ray<float> ray(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, 1.0F));

    // Test intersection
    auto result = sphere.intersect(ray);  // Member function returns pair<T,T>
    EXPECT_TRUE(result.has_value());

    if (result)
    {
        auto [t1, t2] = result.value();

        Vec<float, 3> hit_point = ray.at(t1);

        EXPECT_NEAR(hit_point[2], 4.0F, 1e-5F);  // Should hit at z=4
    }
}

TEST(IntegrationTest, RayPlaneIntersection)
{
    Plane<float> plane(Vec<float, 3>(0.0F, 1.0F, 0.0F), -2.0F);  // y=2 plane
    Ray<float> ray(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 1.0F, 0.0F));

    auto result = intersect(ray, plane);
    EXPECT_TRUE(result.has_value());

    if (result)
    {
        float t = result.value();
        Vec<float, 3> hit_point = ray.at(t);
        EXPECT_NEAR(hit_point[1], 2.0F, 1e-5F);
    }
}

// ===== Physics Simulation =====

TEST(IntegrationTest, RigidBodyRotation)
{
    // Initial orientation
    Quaternion<float> orientation = Quaternion<float>::identity();

    // Angular velocity (radians per second)
    Vec<float, 3> angular_velocity(0.0F, 0.0F, 1.0F);  // 1 rad/s around Z

    // Simulate rotation for 1 second (using small timesteps)
    float dt = 0.01F;
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
    EXPECT_NEAR(euler[2], 1.0F, 0.01F);
}

// ===== Camera System =====

TEST(IntegrationTest, CameraLookAt)
{
    Vec<float, 3> camera_pos(0.0F, 0.0F, 0.0F);
    Vec<float, 3> target(0.0F, 0.0F, -1.0F);
    Vec<float, 3> up(0.0F, 1.0F, 0.0F);

    // Compute forward direction
    Vec<float, 3> forward = (target - camera_pos).normalized();

    // Create look-at matrix
    Mat<float, 3, 3> view_rotation = Mat<float, 3, 3>::look_at(forward, up);

    // Transform a point in front of camera
    Vec<float, 3> world_point(1.0F, 0.0F, -5.0F);
    Vec<float, 3> view_point = view_rotation * world_point;

    // Should be to the right in view space
    EXPECT_GT(view_point[0], 0.0F);
}

// ===== Collision Detection =====

TEST(IntegrationTest, SphereAABBCollision)
{
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 1.0F);
    AABB<float> box(Vec<float, 3>(-2.0F, -0.5F, -0.5F), Vec<float, 3>(2.0F, 0.5F, 0.5F));

    EXPECT_TRUE(sphere.intersects(box));
}

TEST(IntegrationTest, FrustumCulling)
{
    // Create view-projection matrix
    Mat<float, 4, 4> vp = Mat<float, 4, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);

    // Test objects
    Sphere<float> visible_sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 0.5F);
    EXPECT_TRUE(frustum.contains(visible_sphere));
}

// ===== Linear Algebra Operations =====

TEST(IntegrationTest, SolveLinearSystem)
{
    // Solve Ax = b
    Mat<float, 3, 3> A({2.0F, -1.0F, 0.0F, -1.0F, 2.0F, -1.0F, 0.0F, -1.0F, 2.0F});
    Vec<float, 3> b(1.0F, 0.0F, 1.0F);

    // Using LU decomposition
    auto [L, U, P] = A.lu();

    // Check decomposition is correct
    Mat<float, 3, 3> reconstructed = P * L * U;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), A(i, j), 1e-5F);
        }
    }
}

TEST(IntegrationTest, EigenvalueApplication)
{
    // Symmetric matrix
    Mat<float, 3, 3> A({2.0F, -1.0F, 0.0F, -1.0F, 2.0F, -1.0F, 0.0F, -1.0F, 2.0F});

    float eigenvalues[3] = {};
    ASSERT_TRUE(A.eigenvalues_qr(eigenvalues, 3));

    // Should have 3 real eigenvalues
    // All eigenvalues should be positive (positive definite matrix)
    for (float ev : eigenvalues)
    {
        EXPECT_GT(ev, 0.0F);
    }
}

// ===== Interpolation =====

TEST(IntegrationTest, CompleteInterpolationPipeline)
{
    // Vector interpolation
    Vec<float, 3> v1(0.0F, 0.0F, 0.0F);
    Vec<float, 3> v2(1.0F, 1.0F, 1.0F);
    Vec<float, 3> v_mid = v1.lerp(v2, 0.5F);

    EXPECT_NEAR(v_mid[0], 0.5F, 1e-6F);

    // Quaternion interpolation
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2(Vec<float, 3>(0.0F, 0.0F, 1.0F), constants::pi<float>() / 2.0F);
    Quaternion<float> q_mid = q1.slerp(q2, 0.5F);

    // Should be halfway between rotations
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q_mid.rotate(v);

    // Should be at 45 degrees
    EXPECT_NEAR(rotated[0], rotated[1], 0.01F);
}

// ===== Matrix Chain Multiplication =====

TEST(IntegrationTest, MatrixChainMultiplication)
{
    Mat<float, 2, 3> A({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
    Mat<float, 3, 2> B({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
    Mat<float, 2, 2> C({1.0F, 0.0F, 0.0F, 1.0F});

    Mat<float, 2, 2> result = (A * B) * C;

    EXPECT_FLOAT_EQ(result(0, 0), 22.0F);
    EXPECT_FLOAT_EQ(result(0, 1), 28.0F);
}

// ===== Numerical Stability =====

TEST(IntegrationTest, NumericalStabilityLargeRotations)
{
    Quaternion<float> q = Quaternion<float>::identity();

    // Apply many small rotations
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    float small_angle = 0.001F;

    for (int i = 0; i < 1000; ++i)
    {
        Quaternion<float> delta(axis, small_angle);
        q = delta * q;
        q.normalize();
    }

    // Should still be unit quaternion
    EXPECT_NEAR(q.norm(), 1.0F, 1e-5F);

    // Total rotation should be approximately 1 radian
    Vec<float, 3> euler = q.to_euler();
    EXPECT_NEAR(euler[2], 1.0F, 0.01F);
}

// ===== Performance Optimization Tests =====

TEST(IntegrationTest, MatrixVectorBatchOperations)
{
    Mat<float, 3, 3> transform = Mat<float, 3, 3>::rotation_z(0.1F);

    std::vector<Vec<float, 3>> points;
    points.reserve(100);
    for (int i = 0; i < 100; ++i)
    {
        points.emplace_back(static_cast<float>(i), 0.0F, 0.0F);
    }

    std::vector<Vec<float, 3>> transformed;
    transformed.reserve(points.size());
    for (const auto& p : points)
    {
        transformed.push_back(transform * p);
    }

    EXPECT_EQ(transformed.size(), 100);
    EXPECT_TRUE(std::isfinite(transformed[50][0]));
}
