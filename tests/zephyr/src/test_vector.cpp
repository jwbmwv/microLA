// SPDX-License-Identifier: Apache-2.0
/// @file test_vector.cpp
/// @brief Zephyr ztest tests for Vec<T,N> class
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

// Test vectors (using static to persist across test cases)
static Vec<float, 2> v2_zero(0.0F, 0.0F);
static Vec<float, 2> v2_ones(1.0F, 1.0F);
static Vec<float, 2> v2_a(3.0F, 4.0F);
static Vec<float, 2> v2_b(1.0F, 2.0F);

static Vec<float, 3> v3_zero(0.0F, 0.0F, 0.0F);
static Vec<float, 3> v3_unit_x(1.0F, 0.0F, 0.0F);
static Vec<float, 3> v3_unit_y(0.0F, 1.0F, 0.0F);
static Vec<float, 3> v3_unit_z(0.0F, 0.0F, 1.0F);
static Vec<float, 3> v3_a(1.0F, 2.0F, 3.0F);
static Vec<float, 3> v3_b(4.0F, 5.0F, 6.0F);

static Vec<float, 4> v4_a(1.0F, 2.0F, 3.0F, 4.0F);

static const float epsilon = 1e-6F;

// Helper function for floating point comparison
static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

// ===== Construction Tests =====

ZTEST(microla_vector, test_default_constructor)
{
    Vec<float, 3> v;
    zassert_true(float_eq(v[0], 0.0F), "Default constructor should initialize to zero");
    zassert_true(float_eq(v[1], 0.0F), "Default constructor should initialize to zero");
    zassert_true(float_eq(v[2], 0.0F), "Default constructor should initialize to zero");
}

ZTEST(microla_vector, test_variadic_constructor)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    zassert_true(float_eq(v[0], 1.0F), "Variadic constructor x failed");
    zassert_true(float_eq(v[1], 2.0F), "Variadic constructor y failed");
    zassert_true(float_eq(v[2], 3.0F), "Variadic constructor z failed");
}

ZTEST(microla_vector, test_copy_constructor)
{
    Vec<float, 3> v1(1.0F, 2.0F, 3.0F);
    Vec<float, 3> v2(v1);
    zassert_true(v1 == v2, "Copy constructor failed");
}

ZTEST(microla_vector, test_array_constructor)
{
    float arr[] = {1.0F, 2.0F, 3.0F};
    Vec<float, 3> v(arr);
    zassert_true(float_eq(v[0], 1.0F), "Array constructor x failed");
    zassert_true(float_eq(v[1], 2.0F), "Array constructor y failed");
    zassert_true(float_eq(v[2], 3.0F), "Array constructor z failed");
}

// ===== Accessor Tests =====

ZTEST(microla_vector, test_subscript_operator)
{
    zassert_true(float_eq(v3_a[0], 1.0F), "Subscript [0] failed");
    zassert_true(float_eq(v3_a[1], 2.0F), "Subscript [1] failed");
    zassert_true(float_eq(v3_a[2], 3.0F), "Subscript [2] failed");
}

ZTEST(microla_vector, test_component_accessors)
{
    zassert_true(float_eq(v3_a.x(), 1.0F), "x() accessor failed");
    zassert_true(float_eq(v3_a.y(), 2.0F), "y() accessor failed");
    zassert_true(float_eq(v3_a.z(), 3.0F), "z() accessor failed");
    zassert_true(float_eq(v4_a.w(), 4.0F), "w() accessor failed");
}

ZTEST(microla_vector, test_size_method)
{
    zassert_equal(v2_a.size(), 2, "Vec2 size should be 2");
    zassert_equal(v3_a.size(), 3, "Vec3 size should be 3");
    zassert_equal(v4_a.size(), 4, "Vec4 size should be 4");
}

// ===== Arithmetic Operations =====

ZTEST(microla_vector, test_addition)
{
    Vec<float, 3> result = v3_a + v3_b;
    zassert_true(float_eq(result[0], 5.0F), "Addition x failed");
    zassert_true(float_eq(result[1], 7.0F), "Addition y failed");
    zassert_true(float_eq(result[2], 9.0F), "Addition z failed");
}

ZTEST(microla_vector, test_subtraction)
{
    Vec<float, 3> result = v3_b - v3_a;
    zassert_true(float_eq(result[0], 3.0F), "Subtraction x failed");
    zassert_true(float_eq(result[1], 3.0F), "Subtraction y failed");
    zassert_true(float_eq(result[2], 3.0F), "Subtraction z failed");
}

ZTEST(microla_vector, test_scalar_multiplication)
{
    Vec<float, 3> result = v3_a * 2.0F;
    zassert_true(float_eq(result[0], 2.0F), "Scalar mult x failed");
    zassert_true(float_eq(result[1], 4.0F), "Scalar mult y failed");
    zassert_true(float_eq(result[2], 6.0F), "Scalar mult z failed");
}

ZTEST(microla_vector, test_scalar_division)
{
    Vec<float, 3> result = v3_a / 2.0F;
    zassert_true(float_eq(result[0], 0.5F), "Scalar div x failed");
    zassert_true(float_eq(result[1], 1.0F), "Scalar div y failed");
    zassert_true(float_eq(result[2], 1.5F), "Scalar div z failed");
}

// ===== Vector Operations =====

ZTEST(microla_vector, test_dot_product)
{
    float result = v3_a.dot(v3_b);
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    zassert_true(float_eq(result, 32.0F), "Dot product failed");
}

ZTEST(microla_vector, test_cross_product)
{
    Vec<float, 3> result = v3_unit_x.cross(v3_unit_y);
    zassert_true(float_eq(result[0], 0.0F), "Cross product x failed");
    zassert_true(float_eq(result[1], 0.0F), "Cross product y failed");
    zassert_true(float_eq(result[2], 1.0F), "Cross product z failed");
}

ZTEST(microla_vector, test_magnitude)
{
    float mag = v2_a.magnitude();
    // sqrt(3^2 + 4^2) = sqrt(9 + 16) = sqrt(25) = 5
    zassert_true(float_eq(mag, 5.0F), "Magnitude failed");
}

ZTEST(microla_vector, test_normalize)
{
    Vec<float, 3> normalized = v3_a.normalized();
    float mag = normalized.magnitude();
    zassert_true(float_eq(mag, 1.0F), "Normalized vector should have magnitude 1");
}

ZTEST(microla_vector, test_magnitude_squared)
{
    float mag_sq = v2_a.magnitude_squared();
    // 3^2 + 4^2 = 9 + 16 = 25
    zassert_true(float_eq(mag_sq, 25.0F), "Magnitude squared failed");
}

// ===== Comparison Operations =====

ZTEST(microla_vector, test_equality)
{
    Vec<float, 3> v1(1.0F, 2.0F, 3.0F);
    Vec<float, 3> v2(1.0F, 2.0F, 3.0F);
    zassert_true(v1 == v2, "Equality operator failed");
}

ZTEST(microla_vector, test_inequality)
{
    zassert_true(v3_a != v3_b, "Inequality operator failed");
}

// ===== Compound Assignment =====

ZTEST(microla_vector, test_add_assign)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    v += Vec<float, 3>(1.0F, 1.0F, 1.0F);
    zassert_true(float_eq(v[0], 2.0F), "Add assign x failed");
    zassert_true(float_eq(v[1], 3.0F), "Add assign y failed");
    zassert_true(float_eq(v[2], 4.0F), "Add assign z failed");
}

ZTEST(microla_vector, test_subtract_assign)
{
    Vec<float, 3> v(4.0F, 5.0F, 6.0F);
    v -= Vec<float, 3>(1.0F, 1.0F, 1.0F);
    zassert_true(float_eq(v[0], 3.0F), "Subtract assign x failed");
    zassert_true(float_eq(v[1], 4.0F), "Subtract assign y failed");
    zassert_true(float_eq(v[2], 5.0F), "Subtract assign z failed");
}

ZTEST(microla_vector, test_multiply_assign)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    v *= 2.0F;
    zassert_true(float_eq(v[0], 2.0F), "Multiply assign x failed");
    zassert_true(float_eq(v[1], 4.0F), "Multiply assign y failed");
    zassert_true(float_eq(v[2], 6.0F), "Multiply assign z failed");
}

// Test suite setup
ZTEST_SUITE(microla_vector, NULL, NULL, NULL, NULL, NULL);
