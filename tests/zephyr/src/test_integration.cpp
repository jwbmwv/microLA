// SPDX-License-Identifier: MIT
/// @file test_integration.cpp
/// @brief Zephyr ztest integration tests for MicroLA
/// @copyright Copyright (c) 2026 James Baldwin

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

static const float epsilon = 1e-4f;
static const float PI = 3.14159265358979323846f;

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
    Vec<float, 3> point(1.0f, 0.0f, 0.0f);

    // 2. Create a rotation quaternion (90 degrees around Z)
    Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
    Quaternion<float> rotation = Quaternion<float>::from_axis_angle(axis, PI / 2.0f);

    // 3. Apply rotation
    Vec<float, 3> rotated = rotation.rotate(point);

    // Should be approximately (0, 1, 0)
    zassert_true(float_eq(rotated.x(), 0.0f), "Transform pipeline x failed");
    zassert_true(float_eq(rotated.y(), 1.0f), "Transform pipeline y failed");
    zassert_true(float_eq(rotated.z(), 0.0f), "Transform pipeline z failed");

    // 4. Apply scaling
    Vec<float, 3> scaled = rotated * 2.0f;

    // Should be approximately (0, 2, 0)
    zassert_true(float_eq(scaled.x(), 0.0f), "Scaled x failed");
    zassert_true(float_eq(scaled.y(), 2.0f), "Scaled y failed");
    zassert_true(float_eq(scaled.z(), 0.0f), "Scaled z failed");
}

ZTEST(microla_integration, test_coordinate_system_transformation)
{
    // Test converting between coordinate systems using matrices

    // Create a simple 2D rotation matrix (45 degrees)
    float angle = PI / 4.0f;  // 45 degrees
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);

    Mat<float, 2, 2> rotation{cos_a, -sin_a,
                              sin_a,  cos_a};

    // Rotate unit X vector
    Vec<float, 2> v(1.0f, 0.0f);
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
    // Link 2: length 1.0, angle 90°

    float l1 = 1.0f;
    float l2 = 1.0f;
    float theta1 = 0.0f;
    float theta2 = PI / 2.0f;

    // End effector position
    Vec<float, 2> end_effector;
    end_effector[0] = l1 * std::cos(theta1) + l2 * std::cos(theta1 + theta2);
    end_effector[1] = l1 * std::sin(theta1) + l2 * std::sin(theta1 + theta2);

    // Should be approximately (1.0, 1.0)
    zassert_true(float_eq(end_effector.x(), 1.0f), "Forward kinematics x failed");
    zassert_true(float_eq(end_effector.y(), 1.0f), "Forward kinematics y failed");
}

ZTEST(microla_integration, test_quaternion_chain)
{
    // Test chaining multiple rotations
    Vec<float, 3> axis_z(0.0f, 0.0f, 1.0f);
    Vec<float, 3> axis_y(0.0f, 1.0f, 0.0f);

    // 90 degree rotation around Z
    Quaternion<float> q1 = Quaternion<float>::from_axis_angle(axis_z, PI / 2.0f);

    // 90 degree rotation around Y
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(axis_y, PI / 2.0f);

    // Combined rotation
    Quaternion<float> combined = q2 * q1;

    // Apply to unit X vector
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> result = combined.rotate(v);

    // Result should be a valid rotated vector
    float mag = result.magnitude();
    zassert_true(float_eq(mag, 1.0f), "Quaternion chain should preserve magnitude");
}

// ===== Sensor Fusion Integration Tests =====

ZTEST(microla_integration, test_weighted_average)
{
    // Simulate sensor fusion by weighted averaging
    Vec<float, 3> sensor1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> sensor2(2.0f, 3.0f, 4.0f);

    float weight1 = 0.7f;
    float weight2 = 0.3f;

    Vec<float, 3> fused = sensor1 * weight1 + sensor2 * weight2;

    // Expected: (1.0*0.7 + 2.0*0.3, 2.0*0.7 + 3.0*0.3, 3.0*0.7 + 4.0*0.3)
    //         = (1.3, 2.3, 3.3)
    zassert_true(float_eq(fused.x(), 1.3f), "Sensor fusion x failed");
    zassert_true(float_eq(fused.y(), 2.3f), "Sensor fusion y failed");
    zassert_true(float_eq(fused.z(), 3.3f), "Sensor fusion z failed");
}

ZTEST(microla_integration, test_motion_prediction)
{
    // Simple linear motion prediction: new_pos = pos + velocity * dt
    Vec<float, 3> position(1.0f, 2.0f, 3.0f);
    Vec<float, 3> velocity(0.5f, 1.0f, 1.5f);
    float dt = 2.0f;

    Vec<float, 3> predicted = position + velocity * dt;

    // Expected: (1 + 0.5*2, 2 + 1.0*2, 3 + 1.5*2) = (2, 4, 6)
    zassert_true(float_eq(predicted.x(), 2.0f), "Motion prediction x failed");
    zassert_true(float_eq(predicted.y(), 4.0f), "Motion prediction y failed");
    zassert_true(float_eq(predicted.z(), 6.0f), "Motion prediction z failed");
}

// ===== Physics Integration Tests =====

ZTEST(microla_integration, test_reflection_vector)
{
    // Test vector reflection (like light bouncing off a surface)
    // R = V - 2(V·N)N, where N is the surface normal

    Vec<float, 3> incident(1.0f, -1.0f, 0.0f);  // Ray coming down at 45°
    Vec<float, 3> normal(0.0f, 1.0f, 0.0f);     // Horizontal surface (up)

    float dot_vn = incident.dot(normal);
    Vec<float, 3> reflected = incident - normal * (2.0f * dot_vn);

    // Should reflect symmetrically
    zassert_true(float_eq(reflected.x(), 1.0f), "Reflection x failed");
    zassert_true(float_eq(reflected.y(), 1.0f), "Reflection y failed");
    zassert_true(float_eq(reflected.z(), 0.0f), "Reflection z failed");
}

ZTEST(microla_integration, test_projectile_motion)
{
    // Calculate projectile position: p = p0 + v*t + 0.5*a*t^2
    Vec<float, 3> p0(0.0f, 0.0f, 0.0f);         // Initial position
    Vec<float, 3> v(1.0f, 1.0f, 0.0f);          // Initial velocity
    Vec<float, 3> a(0.0f, -9.8f, 0.0f);         // Gravity
    float t = 1.0f;                              // Time

    Vec<float, 3> position = p0 + v * t + a * (0.5f * t * t);

    // Expected: (1.0, 1.0 - 4.9, 0.0) = (1.0, -3.9, 0.0)
    zassert_true(float_eq(position.x(), 1.0f), "Projectile x failed");
    zassert_true(float_eq(position.y(), -3.9f), "Projectile y failed");
    zassert_true(float_eq(position.z(), 0.0f), "Projectile z failed");
}

// ===== Matrix-Vector Combined Operations =====

ZTEST(microla_integration, test_matrix_transform_chain)
{
    // Test applying multiple matrix transformations

    // Create scaling matrix (2x scale)
    Mat<float, 3, 3> scale{2.0f, 0.0f, 0.0f,
                           0.0f, 2.0f, 0.0f,
                           0.0f, 0.0f, 2.0f};

    // Create rotation matrix (identity for simplicity)
    auto rotation = Mat<float, 3, 3>::identity();

    // Combine transformations
    Mat<float, 3, 3> combined = scale * rotation;

    // Apply to vector
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    Vec<float, 3> result = combined * v;

    // Should be scaled by 2
    zassert_true(float_eq(result.x(), 2.0f), "Transform chain x failed");
    zassert_true(float_eq(result.y(), 4.0f), "Transform chain y failed");
    zassert_true(float_eq(result.z(), 6.0f), "Transform chain z failed");
}

ZTEST(microla_integration, test_orthonormal_basis)
{
    // Verify orthonormal basis properties
    Vec<float, 3> u(1.0f, 0.0f, 0.0f);
    Vec<float, 3> v(0.0f, 1.0f, 0.0f);
    Vec<float, 3> w = u.cross(v);

    // Check orthogonality
    zassert_true(float_eq(u.dot(v), 0.0f), "Basis vectors not orthogonal");
    zassert_true(float_eq(u.dot(w), 0.0f), "Basis vectors not orthogonal");
    zassert_true(float_eq(v.dot(w), 0.0f), "Basis vectors not orthogonal");

    // Check normalization
    zassert_true(float_eq(u.magnitude(), 1.0f), "Basis vector not normalized");
    zassert_true(float_eq(v.magnitude(), 1.0f), "Basis vector not normalized");
    zassert_true(float_eq(w.magnitude(), 1.0f), "Basis vector not normalized");

    // w should be unit Z
    zassert_true(float_eq(w.z(), 1.0f), "Cross product failed to produce unit Z");
}

// ===== Performance/Stress Tests =====

ZTEST(microla_integration, test_multiple_operations)
{
    // Test a series of operations to ensure consistency
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);

    // Apply multiple operations
    v = v * 2.0f;
    v = v + Vec<float, 3>(1.0f, 1.0f, 1.0f);
    v = v / 2.0f;

    // Expected: ((1,2,3)*2 + (1,1,1))/2 = ((2,4,6) + (1,1,1))/2 = (3,5,7)/2 = (1.5, 2.5, 3.5)
    zassert_true(float_eq(v.x(), 1.5f), "Multiple ops x failed");
    zassert_true(float_eq(v.y(), 2.5f), "Multiple ops y failed");
    zassert_true(float_eq(v.z(), 3.5f), "Multiple ops z failed");
}

ZTEST(microla_integration, test_constexpr_operations)
{
    // Test compile-time computations
#if defined(__ZEPHYR__)
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(4.0f, 5.0f, 6.0f);
    Vec<float, 3> sum = v1 + v2;
#else
    constexpr Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    constexpr Vec<float, 3> v2(4.0f, 5.0f, 6.0f);
    constexpr Vec<float, 3> sum = v1 + v2;
#endif

    zassert_true(float_eq(sum.x(), 5.0f), "Constexpr x failed");
    zassert_true(float_eq(sum.y(), 7.0f), "Constexpr y failed");
    zassert_true(float_eq(sum.z(), 9.0f), "Constexpr z failed");
}

// Test suite setup
ZTEST_SUITE(microla_integration, NULL, NULL, NULL, NULL, NULL);
