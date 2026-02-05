// SPDX-License-Identifier: MIT
/// @file test_vector.cpp
/// @brief Zephyr ztest tests for Vec<T,N> class
/// @copyright Copyright (c) 2026 James Baldwin

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

// Test vectors (using static to persist across test cases)
static Vec<float, 2> v2_zero(0.0f, 0.0f);
static Vec<float, 2> v2_ones(1.0f, 1.0f);
static Vec<float, 2> v2_a(3.0f, 4.0f);
static Vec<float, 2> v2_b(1.0f, 2.0f);

static Vec<float, 3> v3_zero(0.0f, 0.0f, 0.0f);
static Vec<float, 3> v3_unit_x(1.0f, 0.0f, 0.0f);
static Vec<float, 3> v3_unit_y(0.0f, 1.0f, 0.0f);
static Vec<float, 3> v3_unit_z(0.0f, 0.0f, 1.0f);
static Vec<float, 3> v3_a(1.0f, 2.0f, 3.0f);
static Vec<float, 3> v3_b(4.0f, 5.0f, 6.0f);

static Vec<float, 4> v4_a(1.0f, 2.0f, 3.0f, 4.0f);

static const float epsilon = 1e-6f;

// Helper function for floating point comparison
static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

// ===== Construction Tests =====

ZTEST(microla_vector, test_default_constructor)
{
    Vec<float, 3> v;
    zassert_true(float_eq(v[0], 0.0f), "Default constructor should initialize to zero");
    zassert_true(float_eq(v[1], 0.0f), "Default constructor should initialize to zero");
    zassert_true(float_eq(v[2], 0.0f), "Default constructor should initialize to zero");
}

ZTEST(microla_vector, test_variadic_constructor)
{
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    zassert_true(float_eq(v[0], 1.0f), "Variadic constructor x failed");
    zassert_true(float_eq(v[1], 2.0f), "Variadic constructor y failed");
    zassert_true(float_eq(v[2], 3.0f), "Variadic constructor z failed");
}

ZTEST(microla_vector, test_copy_constructor)
{
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(v1);
    zassert_true(v1 == v2, "Copy constructor failed");
}

ZTEST(microla_vector, test_array_constructor)
{
    float arr[] = {1.0f, 2.0f, 3.0f};
    Vec<float, 3> v(arr);
    zassert_true(float_eq(v[0], 1.0f), "Array constructor x failed");
    zassert_true(float_eq(v[1], 2.0f), "Array constructor y failed");
    zassert_true(float_eq(v[2], 3.0f), "Array constructor z failed");
}

// ===== Accessor Tests =====

ZTEST(microla_vector, test_subscript_operator)
{
    zassert_true(float_eq(v3_a[0], 1.0f), "Subscript [0] failed");
    zassert_true(float_eq(v3_a[1], 2.0f), "Subscript [1] failed");
    zassert_true(float_eq(v3_a[2], 3.0f), "Subscript [2] failed");
}

ZTEST(microla_vector, test_component_accessors)
{
    zassert_true(float_eq(v3_a.x(), 1.0f), "x() accessor failed");
    zassert_true(float_eq(v3_a.y(), 2.0f), "y() accessor failed");
    zassert_true(float_eq(v3_a.z(), 3.0f), "z() accessor failed");
    zassert_true(float_eq(v4_a.w(), 4.0f), "w() accessor failed");
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
    zassert_true(float_eq(result[0], 5.0f), "Addition x failed");
    zassert_true(float_eq(result[1], 7.0f), "Addition y failed");
    zassert_true(float_eq(result[2], 9.0f), "Addition z failed");
}

ZTEST(microla_vector, test_subtraction)
{
    Vec<float, 3> result = v3_b - v3_a;
    zassert_true(float_eq(result[0], 3.0f), "Subtraction x failed");
    zassert_true(float_eq(result[1], 3.0f), "Subtraction y failed");
    zassert_true(float_eq(result[2], 3.0f), "Subtraction z failed");
}

ZTEST(microla_vector, test_scalar_multiplication)
{
    Vec<float, 3> result = v3_a * 2.0f;
    zassert_true(float_eq(result[0], 2.0f), "Scalar mult x failed");
    zassert_true(float_eq(result[1], 4.0f), "Scalar mult y failed");
    zassert_true(float_eq(result[2], 6.0f), "Scalar mult z failed");
}

ZTEST(microla_vector, test_scalar_division)
{
    Vec<float, 3> result = v3_a / 2.0f;
    zassert_true(float_eq(result[0], 0.5f), "Scalar div x failed");
    zassert_true(float_eq(result[1], 1.0f), "Scalar div y failed");
    zassert_true(float_eq(result[2], 1.5f), "Scalar div z failed");
}

// ===== Vector Operations =====

ZTEST(microla_vector, test_dot_product)
{
    float result = v3_a.dot(v3_b);
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    zassert_true(float_eq(result, 32.0f), "Dot product failed");
}

ZTEST(microla_vector, test_cross_product)
{
    Vec<float, 3> result = v3_unit_x.cross(v3_unit_y);
    zassert_true(float_eq(result[0], 0.0f), "Cross product x failed");
    zassert_true(float_eq(result[1], 0.0f), "Cross product y failed");
    zassert_true(float_eq(result[2], 1.0f), "Cross product z failed");
}

ZTEST(microla_vector, test_magnitude)
{
    float mag = v2_a.magnitude();
    // sqrt(3^2 + 4^2) = sqrt(9 + 16) = sqrt(25) = 5
    zassert_true(float_eq(mag, 5.0f), "Magnitude failed");
}

ZTEST(microla_vector, test_normalize)
{
    Vec<float, 3> normalized = v3_a.normalized();
    float mag = normalized.magnitude();
    zassert_true(float_eq(mag, 1.0f), "Normalized vector should have magnitude 1");
}

ZTEST(microla_vector, test_magnitude_squared)
{
    float mag_sq = v2_a.magnitude_squared();
    // 3^2 + 4^2 = 9 + 16 = 25
    zassert_true(float_eq(mag_sq, 25.0f), "Magnitude squared failed");
}

// ===== Comparison Operations =====

ZTEST(microla_vector, test_equality)
{
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(1.0f, 2.0f, 3.0f);
    zassert_true(v1 == v2, "Equality operator failed");
}

ZTEST(microla_vector, test_inequality)
{
    zassert_true(v3_a != v3_b, "Inequality operator failed");
}

// ===== Compound Assignment =====

ZTEST(microla_vector, test_add_assign)
{
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    v += Vec<float, 3>(1.0f, 1.0f, 1.0f);
    zassert_true(float_eq(v[0], 2.0f), "Add assign x failed");
    zassert_true(float_eq(v[1], 3.0f), "Add assign y failed");
    zassert_true(float_eq(v[2], 4.0f), "Add assign z failed");
}

ZTEST(microla_vector, test_subtract_assign)
{
    Vec<float, 3> v(4.0f, 5.0f, 6.0f);
    v -= Vec<float, 3>(1.0f, 1.0f, 1.0f);
    zassert_true(float_eq(v[0], 3.0f), "Subtract assign x failed");
    zassert_true(float_eq(v[1], 4.0f), "Subtract assign y failed");
    zassert_true(float_eq(v[2], 5.0f), "Subtract assign z failed");
}

ZTEST(microla_vector, test_multiply_assign)
{
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    v *= 2.0f;
    zassert_true(float_eq(v[0], 2.0f), "Multiply assign x failed");
    zassert_true(float_eq(v[1], 4.0f), "Multiply assign y failed");
    zassert_true(float_eq(v[2], 6.0f), "Multiply assign z failed");
}

// Test suite setup
ZTEST_SUITE(microla_vector, NULL, NULL, NULL, NULL, NULL);
