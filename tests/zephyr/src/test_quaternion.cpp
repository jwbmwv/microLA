// SPDX-License-Identifier: Apache-2.0
/// @file test_quaternion.cpp
/// @brief Zephyr ztest tests for Quaternion<T> class
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

static const float epsilon = 1e-5F;
static const float PI = 3.14159265358979323846F;

// Helper function for floating point comparison
static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

// ===== Construction Tests =====

ZTEST(microla_quaternion, test_default_constructor)
{
    Quaternion<float> q;

    // Default should be identity quaternion (1, 0, 0, 0)
    zassert_true(float_eq(q.w(), 1.0F), "Default w should be 1");
    zassert_true(float_eq(q.x(), 0.0F), "Default x should be 0");
    zassert_true(float_eq(q.y(), 0.0F), "Default y should be 0");
    zassert_true(float_eq(q.z(), 0.0F), "Default z should be 0");
}

ZTEST(microla_quaternion, test_component_constructor)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);

    zassert_true(float_eq(q.w(), 1.0F), "Component constructor w failed");
    zassert_true(float_eq(q.x(), 2.0F), "Component constructor x failed");
    zassert_true(float_eq(q.y(), 3.0F), "Component constructor y failed");
    zassert_true(float_eq(q.z(), 4.0F), "Component constructor z failed");
}

ZTEST(microla_quaternion, test_copy_constructor)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(q1);

    zassert_true(q1 == q2, "Copy constructor failed");
}

// ===== Identity and Special Quaternions =====

ZTEST(microla_quaternion, test_identity)
{
    auto q = Quaternion<float>::identity();

    zassert_true(float_eq(q.w(), 1.0F), "Identity w should be 1");
    zassert_true(float_eq(q.x(), 0.0F), "Identity x should be 0");
    zassert_true(float_eq(q.y(), 0.0F), "Identity y should be 0");
    zassert_true(float_eq(q.z(), 0.0F), "Identity z should be 0");
}

// ===== Arithmetic Operations =====

ZTEST(microla_quaternion, test_addition)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(5.0F, 6.0F, 7.0F, 8.0F);

    Quaternion<float> result = q1 + q2;

    zassert_true(float_eq(result.w(), 6.0F), "Addition w failed");
    zassert_true(float_eq(result.x(), 8.0F), "Addition x failed");
    zassert_true(float_eq(result.y(), 10.0F), "Addition y failed");
    zassert_true(float_eq(result.z(), 12.0F), "Addition z failed");
}

ZTEST(microla_quaternion, test_subtraction)
{
    Quaternion<float> q1(5.0F, 6.0F, 7.0F, 8.0F);
    Quaternion<float> q2(1.0F, 2.0F, 3.0F, 4.0F);

    Quaternion<float> result = q1 - q2;

    zassert_true(float_eq(result.w(), 4.0F), "Subtraction w failed");
    zassert_true(float_eq(result.x(), 4.0F), "Subtraction x failed");
    zassert_true(float_eq(result.y(), 4.0F), "Subtraction y failed");
    zassert_true(float_eq(result.z(), 4.0F), "Subtraction z failed");
}

ZTEST(microla_quaternion, test_scalar_multiplication)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> result = q * 2.0F;

    zassert_true(float_eq(result.w(), 2.0F), "Scalar mult w failed");
    zassert_true(float_eq(result.x(), 4.0F), "Scalar mult x failed");
    zassert_true(float_eq(result.y(), 6.0F), "Scalar mult y failed");
    zassert_true(float_eq(result.z(), 8.0F), "Scalar mult z failed");
}

ZTEST(microla_quaternion, test_quaternion_multiplication)
{
    // Test with simple quaternions
    Quaternion<float> q1(1.0F, 0.0F, 1.0F, 0.0F);
    Quaternion<float> q2(1.0F, 0.5F, 0.5F, 0.75F);

    Quaternion<float> result = q1 * q2;

    // Quaternion multiplication is non-commutative
    // This test just ensures the operation executes without error
    // and produces a valid quaternion
    float norm_sq =
        result.w() * result.w() + result.x() * result.x() + result.y() * result.y() + result.z() * result.z();

    zassert_true(norm_sq > 0.0F, "Quaternion multiplication produced invalid result");
}

// ===== Quaternion Operations =====

ZTEST(microla_quaternion, test_norm)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    float norm = q.norm();

    // sqrt(1 + 4 + 9 + 16) = sqrt(30)
    float expected = std::sqrt(30.0F);
    zassert_true(float_eq(norm, expected), "Norm calculation failed");
}

ZTEST(microla_quaternion, test_normalized)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> normalized = q.normalized();

    float norm = normalized.norm();
    zassert_true(float_eq(norm, 1.0F), "Normalized quaternion should have norm 1");
}

ZTEST(microla_quaternion, test_conjugate)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> conj = q.conjugate();

    zassert_true(float_eq(conj.w(), 1.0F), "Conjugate w should be unchanged");
    zassert_true(float_eq(conj.x(), -2.0F), "Conjugate x should be negated");
    zassert_true(float_eq(conj.y(), -3.0F), "Conjugate y should be negated");
    zassert_true(float_eq(conj.z(), -4.0F), "Conjugate z should be negated");
}

ZTEST(microla_quaternion, test_inverse)
{
    Quaternion<float> q(1.0F, 0.0F, 1.0F, 0.0F);
    Quaternion<float> q_inv = q.inverse();
    Quaternion<float> product = q * q_inv;

    // Product should be approximately identity
    zassert_true(float_eq(product.w(), 1.0F, 1e-4F), "Inverse w failed");
    zassert_true(float_eq(product.x(), 0.0F, 1e-4F), "Inverse x failed");
    zassert_true(float_eq(product.y(), 0.0F, 1e-4F), "Inverse y failed");
    zassert_true(float_eq(product.z(), 0.0F, 1e-4F), "Inverse z failed");
}

// ===== Rotation Tests =====

ZTEST(microla_quaternion, test_from_axis_angle)
{
    // 90 degree rotation around Z axis
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    float angle = PI / 2.0F;

    Quaternion<float> q = Quaternion<float>::from_axis_angle(axis, angle);

    // For 90 degrees around Z: w = cos(45°) ≈ 0.707, z = sin(45°) ≈ 0.707
    zassert_true(float_eq(q.w(), std::cos(angle / 2.0F), 1e-4F), "Axis-angle w failed");
    zassert_true(float_eq(q.x(), 0.0F, 1e-4F), "Axis-angle x failed");
    zassert_true(float_eq(q.y(), 0.0F, 1e-4F), "Axis-angle y failed");
    zassert_true(float_eq(q.z(), std::sin(angle / 2.0F), 1e-4F), "Axis-angle z failed");
}

ZTEST(microla_quaternion, test_rotate_vector)
{
    // 90 degree rotation around Z axis
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    float angle = PI / 2.0F;
    Quaternion<float> q = Quaternion<float>::from_axis_angle(axis, angle);

    // Rotate unit X vector
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q.rotate(v);

    // Should become approximately unit Y vector
    zassert_true(float_eq(rotated.x(), 0.0F, 1e-4F), "Rotated x failed");
    zassert_true(float_eq(rotated.y(), 1.0F, 1e-4F), "Rotated y failed");
    zassert_true(float_eq(rotated.z(), 0.0F, 1e-4F), "Rotated z failed");
}

ZTEST(microla_quaternion, test_to_matrix)
{
    // Identity quaternion should produce identity matrix
    auto q = Quaternion<float>::identity();
    auto m = q.to_matrix();

    // Check diagonal elements
    zassert_true(float_eq(m(0, 0), 1.0F, 1e-4F), "Rotation matrix [0,0] failed");
    zassert_true(float_eq(m(1, 1), 1.0F, 1e-4F), "Rotation matrix [1,1] failed");
    zassert_true(float_eq(m(2, 2), 1.0F, 1e-4F), "Rotation matrix [2,2] failed");

    // Check off-diagonal elements
    zassert_true(float_eq(m(0, 1), 0.0F, 1e-4F), "Rotation matrix off-diagonal failed");
    zassert_true(float_eq(m(1, 0), 0.0F, 1e-4F), "Rotation matrix off-diagonal failed");
}

// ===== SLERP Test =====

ZTEST(microla_quaternion, test_slerp)
{
    auto q1 = Quaternion<float>::identity();
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    auto q2 = Quaternion<float>::from_axis_angle(axis, PI / 2.0F);

    // Interpolate half-way
    Quaternion<float> result = q1.slerp(q2, 0.5F);

    // Result should be a valid quaternion with norm ≈ 1
    float norm = result.norm();
    zassert_true(float_eq(norm, 1.0F, 1e-4F), "SLERP result should be normalized");
}

// ===== Comparison Operations =====

ZTEST(microla_quaternion, test_equality)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(1.0F, 2.0F, 3.0F, 4.0F);

    zassert_true(q1 == q2, "Quaternion equality failed");
}

ZTEST(microla_quaternion, test_inequality)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(5.0F, 6.0F, 7.0F, 8.0F);

    zassert_true(q1 != q2, "Quaternion inequality failed");
}

// Test suite setup
ZTEST_SUITE(microla_quaternion, NULL, NULL, NULL, NULL, NULL);
