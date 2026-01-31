// SPDX-License-Identifier: Apache-2.0
/// @file test_integration.cpp
/// @brief Zephyr ztest integration tests for MicroLA
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

static const float epsilon = 1e-4F;
static const float PI = 3.14159265358979323846F;

// Helper function for floating point comparison
static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

// ===== Graphics Pipeline Integration Tests =====

ZTEST(microla_integration, test_transform_pipeline)
{
    // Create a simple graphics transform pipeline:
    // Translation -> Rotation -> Scaling

    // 1. Start with a point
    Vec<float, 3> point(1.0F, 0.0F, 0.0F);

    // 2. Create a rotation quaternion (90 degrees around Z)
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    Quaternion<float> rotation = Quaternion<float>::from_axis_angle(axis, PI / 2.0F);

    // 3. Apply rotation
    Vec<float, 3> rotated = rotation.rotate(point);

    // Should be approximately (0, 1, 0)
    zassert_true(float_eq(rotated.x(), 0.0F), "Transform pipeline x failed");
    zassert_true(float_eq(rotated.y(), 1.0F), "Transform pipeline y failed");
    zassert_true(float_eq(rotated.z(), 0.0F), "Transform pipeline z failed");

    // 4. Apply scaling
    Vec<float, 3> scaled = rotated * 2.0F;

    // Should be approximately (0, 2, 0)
    zassert_true(float_eq(scaled.x(), 0.0F), "Scaled x failed");
    zassert_true(float_eq(scaled.y(), 2.0F), "Scaled y failed");
    zassert_true(float_eq(scaled.z(), 0.0F), "Scaled z failed");
}

ZTEST(microla_integration, test_coordinate_system_transformation)
{
    // Test converting between coordinate systems using matrices

    // Create a simple 2D rotation matrix (45 degrees)
    float angle = PI / 4.0F;  // 45 degrees
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);

    // clang-format off
    Mat<float, 2, 2> rotation({
        {cos_a, -sin_a},
        {sin_a,  cos_a}
    });
    // clang-format on

    // Rotate unit X vector
    Vec<float, 2> v(1.0F, 0.0F);
    Vec<float, 2> rotated = rotation * v;

    // Should be approximately (0.707, 0.707)
    zassert_true(float_eq(rotated.x(), cos_a), "Coord transform x failed");
    zassert_true(float_eq(rotated.y(), sin_a), "Coord transform y failed");
}

// ===== Robotics Kinematics Integration Tests =====

ZTEST(microla_integration, test_forward_kinematics)
{
    // Simple 2-link planar robot arm forward kinematics
    // Link 1: length 1.0, angle 0°
    // Link 2: length 1.0, angle 90°Mat

    float l1 = 1.0F;
    float l2 = 1.0F;
    float theta1 = 0.0F;
    float theta2 = PI / 2.0F;

    // End effector position
    Vec<float, 2> end_effector;
    end_effector[0] = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
    end_effector[1] = l1 * std::sin(theta1) + l2 * std::sin(theta1 + theta2);

    // Should be approximately (1.0, 1.0)
    zassert_true(float_eq(end_effector.x(), 1.0F), "Forward kinematics x failed");
    zassert_true(float_eq(end_effector.y(), 1.0F), "Forward kinematics y failed");
}

ZTEST(microla_integration, test_quaternion_chain)
{
    // Test chaining multiple rotations
    Vec<float, 3> axis_z(0.0F, 0.0F, 1.0F);
    Vec<float, 3> axis_y(0.0F, 1.0F, 0.0F);

    // 90 degree rotation around Z
    Quaternion<float> q1 = Quaternion<float>::from_axis_angle(axis_z, PI / 2.0F);

    // 90 degree rotation around Y
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(axis_y, PI / 2.0F);

    // Combined rotation
    Quaternion<float> combined = q2 * q1;

    // Apply to unit X vector
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> result = combined.rotate(v);

    // Result should be a valid rotated vector
    float mag = result.magnitude();
    zassert_true(float_eq(mag, 1.0F), "Quaternion chain should preserve magnitude");
}

// ===== Sensor Fusion Integration Tests =====

ZTEST(microla_integration, test_weighted_average)
{
    // Simulate sensor fusion by weighted averaging
    Vec<float, 3> sensor1(1.0F, 2.0F, 3.0F);
    Vec<float, 3> sensor2(2.0F, 3.0F, 4.0F);

    float weight1 = 0.7F;
    float weight2 = 0.3F;

    Vec<float, 3> fused = sensor1 * weight1 + sensor2 * weight2;

    // Expected: (1.0*0.7 + 2.0*0.3, 2.0*0.7 + 3.0*0.3, 3.0*0.7 + 4.0*0.3)
    //         = (1.3, 2.3, 3.3)
    zassert_true(float_eq(fused.x(), 1.3F), "Sensor fusion x failed");
    zassert_true(float_eq(fused.y(), 2.3F), "Sensor fusion y failed");
    zassert_true(float_eq(fused.z(), 3.3F), "Sensor fusion z failed");
}

ZTEST(microla_integration, test_motion_prediction)
{
    // Simple linear motion prediction: new_pos = pos + velocity * dt
    Vec<float, 3> position(1.0F, 2.0F, 3.0F);
    Vec<float, 3> velocity(0.5F, 1.0F, 1.5F);
    float dt = 2.0F;

    Vec<float, 3> predicted = position + velocity * dt;

    // Expected: (1 + 0.5*2, 2 + 1.0*2, 3 + 1.5*2) = (2, 4, 6)
    zassert_true(float_eq(predicted.x(), 2.0F), "Motion prediction x failed");
    zassert_true(float_eq(predicted.y(), 4.0F), "Motion prediction y failed");
    zassert_true(float_eq(predicted.z(), 6.0F), "Motion prediction z failed");
}

// ===== Physics Integration Tests =====

ZTEST(microla_integration, test_reflection_vector)
{
    // Test vector reflection (like light bouncing off a surface)
    // R = V - 2(V·N)N, where N is the surface normal

    Vec<float, 3> incident(1.0F, -1.0F, 0.0F);  // Ray coming down at 45°
    Vec<float, 3> normal(0.0F, 1.0F, 0.0F);     // Horizontal surface (up)

    float dot_vn = incident.dot(normal);
    Vec<float, 3> reflected = incident - normal * (2.0F * dot_vn);

    // Should reflect symmetrically
    zassert_true(float_eq(reflected.x(), 1.0F), "Reflection x failed");
    zassert_true(float_eq(reflected.y(), 1.0F), "Reflection y failed");
    zassert_true(float_eq(reflected.z(), 0.0F), "Reflection z failed");
}

ZTEST(microla_integration, test_projectile_motion)
{
    // Calculate projectile position: p = p0 + v*t + 0.5*a*t^2
    Vec<float, 3> p0(0.0F, 0.0F, 0.0F);  // Initial position
    Vec<float, 3> v(1.0F, 1.0F, 0.0F);   // Initial velocity
    Vec<float, 3> a(0.0F, -9.8F, 0.0F);  // Gravity
    float t = 1.0F;                      // Time

    Vec<float, 3> position = p0 + v * t + a * (0.5F * t * t);

    // Expected: (1.0, 1.0 - 4.9, 0.0) = (1.0, -3.9, 0.0)
    zassert_true(float_eq(position.x(), 1.0F), "Projectile x failed");
    zassert_true(float_eq(position.y(), -3.9F), "Projectile y failed");
    zassert_true(float_eq(position.z(), 0.0F), "Projectile z failed");
}

// ===== Matrix-Vector Combined Operations =====

ZTEST(microla_integration, test_matrix_transform_chain)
{
    // Test applying multiple matrix transformations

    // Create scaling matrix (2x scale)
    // clang-format off
    Mat<float, 3, 3> scale({
        {2.0F, 0.0F, 0.0F},
        {0.0F, 2.0F, 0.0F},
        {0.0F, 0.0F, 2.0F}
    });
    // clang-format on

    // Create rotation matrix (identity for simplicity)
    auto rotation = Mat<float, 3, 3>::identity();

    // Combine transformations
    Mat<float, 3, 3> combined = scale * rotation;

    // Apply to vector
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    Vec<float, 3> result = combined * v;

    // Should be scaled by 2
    zassert_true(float_eq(result.x(), 2.0F), "Transform chain x failed");
    zassert_true(float_eq(result.y(), 4.0F), "Transform chain y failed");
    zassert_true(float_eq(result.z(), 6.0F), "Transform chain z failed");
}

ZTEST(microla_integration, test_orthonormal_basis)
{
    // Verify orthonormal basis properties
    Vec<float, 3> u(1.0F, 0.0F, 0.0F);
    Vec<float, 3> v(0.0F, 1.0F, 0.0F);
    Vec<float, 3> w = u.cross(v);

    // Check orthogonality
    zassert_true(float_eq(u.dot(v), 0.0F), "Basis vectors not orthogonal");
    zassert_true(float_eq(u.dot(w), 0.0F), "Basis vectors not orthogonal");
    zassert_true(float_eq(v.dot(w), 0.0F), "Basis vectors not orthogonal");

    // Check normalization
    zassert_true(float_eq(u.magnitude(), 1.0F), "Basis vector not normalized");
    zassert_true(float_eq(v.magnitude(), 1.0F), "Basis vector not normalized");
    zassert_true(float_eq(w.magnitude(), 1.0F), "Basis vector not normalized");

    // w should be unit Z
    zassert_true(float_eq(w.z(), 1.0F), "Cross product failed to produce unit Z");
}

// ===== Performance/Stress Tests =====

ZTEST(microla_integration, test_multiple_operations)
{
    // Test a series of operations to ensure consistency
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);

    // Apply multiple operations
    v = v * 2.0F;
    v = v + Vec<float, 3>(1.0F, 1.0F, 1.0F);
    v = v / 2.0F;

    // Expected: ((1,2,3)*2 + (1,1,1))/2 = ((2,4,6) + (1,1,1))/2 = (3,5,7)/2 = (1.5, 2.5, 3.5)
    zassert_true(float_eq(v.x(), 1.5F), "Multiple ops x failed");
    zassert_true(float_eq(v.y(), 2.5F), "Multiple ops y failed");
    zassert_true(float_eq(v.z(), 3.5F), "Multiple ops z failed");
}

ZTEST(microla_integration, test_constexpr_operations)
{
    // Test compile-time computations
#if defined(__ZEPHYR__)
    Vec<float, 3> v1(1.0F, 2.0F, 3.0F);
    Vec<float, 3> v2(4.0F, 5.0F, 6.0F);
    Vec<float, 3> sum = v1 + v2;
#else
    constexpr Vec<float, 3> v1(1.0F, 2.0F, 3.0F);
    constexpr Vec<float, 3> v2(4.0F, 5.0F, 6.0F);
    constexpr Vec<float, 3> sum = v1 + v2;
#endif

    zassert_true(float_eq(sum.x(), 5.0F), "Constexpr x failed");
    zassert_true(float_eq(sum.y(), 7.0F), "Constexpr y failed");
    zassert_true(float_eq(sum.z(), 9.0F), "Constexpr z failed");
}

// Test suite setup
ZTEST_SUITE(microla_integration, NULL, NULL, NULL, NULL, NULL);
