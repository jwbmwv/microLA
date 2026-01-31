// SPDX-License-Identifier: MIT
/// @file test_vector.cpp
/// @brief Comprehensive tests for Vec<T,N> class
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace microla;

// Test fixture for vector tests
class VectorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Common test data
        v2_zero = Vec<float, 2>(0.0f, 0.0f);
        v2_ones = Vec<float, 2>(1.0f, 1.0f);
        v2_a = Vec<float, 2>(3.0f, 4.0f);
        v2_b = Vec<float, 2>(1.0f, 2.0f);

        v3_zero = Vec<float, 3>(0.0f, 0.0f, 0.0f);
        v3_unit_x = Vec<float, 3>(1.0f, 0.0f, 0.0f);
        v3_unit_y = Vec<float, 3>(0.0f, 1.0f, 0.0f);
        v3_unit_z = Vec<float, 3>(0.0f, 0.0f, 1.0f);
        v3_a = Vec<float, 3>(1.0f, 2.0f, 3.0f);
        v3_b = Vec<float, 3>(4.0f, 5.0f, 6.0f);

        v4_a = Vec<float, 4>(1.0f, 2.0f, 3.0f, 4.0f);
    }

    Vec<float, 2> v2_zero, v2_ones, v2_a, v2_b;
    Vec<float, 3> v3_zero, v3_unit_x, v3_unit_y, v3_unit_z, v3_a, v3_b;
    Vec<float, 4> v4_a;
    const float epsilon = 1e-6f;
};

// ===== Construction Tests =====

TEST_F(VectorTest, DefaultConstructor)
{
    Vec<float, 3> v;
    EXPECT_FLOAT_EQ(v[0], 0.0f);
    EXPECT_FLOAT_EQ(v[1], 0.0f);
    EXPECT_FLOAT_EQ(v[2], 0.0f);
}

TEST_F(VectorTest, VariadicConstructor)
{
    Vec<float, 3> v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[1], 2.0f);
    EXPECT_FLOAT_EQ(v[2], 3.0f);
}

TEST_F(VectorTest, CopyConstructor)
{
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(v1);
    EXPECT_EQ(v1, v2);
}

TEST_F(VectorTest, ArrayConstructor)
{
    float arr[] = {1.0f, 2.0f, 3.0f};
    Vec<float, 3> v(arr);
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[1], 2.0f);
    EXPECT_FLOAT_EQ(v[2], 3.0f);
}

// ===== Accessors Tests =====

TEST_F(VectorTest, SubscriptOperator)
{
    EXPECT_FLOAT_EQ(v3_a[0], 1.0f);
    EXPECT_FLOAT_EQ(v3_a[1], 2.0f);
    EXPECT_FLOAT_EQ(v3_a[2], 3.0f);
}

TEST_F(VectorTest, ComponentAccessors)
{
    EXPECT_FLOAT_EQ(v3_a.x(), 1.0f);
    EXPECT_FLOAT_EQ(v3_a.y(), 2.0f);
    EXPECT_FLOAT_EQ(v3_a.z(), 3.0f);
    EXPECT_FLOAT_EQ(v4_a.w(), 4.0f);
}

TEST_F(VectorTest, SizeMethod)
{
    EXPECT_EQ(v2_a.size(), 2);
    EXPECT_EQ(v3_a.size(), 3);
    EXPECT_EQ(v4_a.size(), 4);
}

// ===== Arithmetic Operations =====

TEST_F(VectorTest, Addition)
{
    Vec<float, 3> result = v3_a + v3_b;
    EXPECT_FLOAT_EQ(result[0], 5.0f);
    EXPECT_FLOAT_EQ(result[1], 7.0f);
    EXPECT_FLOAT_EQ(result[2], 9.0f);
}

TEST_F(VectorTest, Subtraction)
{
    Vec<float, 3> result = v3_b - v3_a;
    EXPECT_FLOAT_EQ(result[0], 3.0f);
    EXPECT_FLOAT_EQ(result[1], 3.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
}

TEST_F(VectorTest, ScalarMultiplication)
{
    Vec<float, 3> result = v3_a * 2.0f;
    EXPECT_FLOAT_EQ(result[0], 2.0f);
    EXPECT_FLOAT_EQ(result[1], 4.0f);
    EXPECT_FLOAT_EQ(result[2], 6.0f);
}

TEST_F(VectorTest, ScalarMultiplicationCommutative)
{
    Vec<float, 3> result = 2.0f * v3_a;
    EXPECT_FLOAT_EQ(result[0], 2.0f);
    EXPECT_FLOAT_EQ(result[1], 4.0f);
    EXPECT_FLOAT_EQ(result[2], 6.0f);
}

TEST_F(VectorTest, ScalarDivision)
{
    Vec<float, 3> result = v3_a / 2.0f;
    EXPECT_FLOAT_EQ(result[0], 0.5f);
    EXPECT_FLOAT_EQ(result[1], 1.0f);
    EXPECT_FLOAT_EQ(result[2], 1.5f);
}

TEST_F(VectorTest, DivisionByZero)
{
    Vec<float, 3> result = v3_a / 0.0f;
    // Should return zero vector for safety
    EXPECT_FLOAT_EQ(result[0], 0.0f);
    EXPECT_FLOAT_EQ(result[1], 0.0f);
    EXPECT_FLOAT_EQ(result[2], 0.0f);
}

TEST_F(VectorTest, UnaryMinus)
{
    Vec<float, 3> result = -v3_a;
    EXPECT_FLOAT_EQ(result[0], -1.0f);
    EXPECT_FLOAT_EQ(result[1], -2.0f);
    EXPECT_FLOAT_EQ(result[2], -3.0f);
}

// ===== Compound Assignment Tests =====

TEST_F(VectorTest, AdditionAssignment)
{
    Vec<float, 3> v = v3_a;
    v += v3_b;
    EXPECT_FLOAT_EQ(v[0], 5.0f);
    EXPECT_FLOAT_EQ(v[1], 7.0f);
    EXPECT_FLOAT_EQ(v[2], 9.0f);
}

TEST_F(VectorTest, SubtractionAssignment)
{
    Vec<float, 3> v = v3_b;
    v -= v3_a;
    EXPECT_FLOAT_EQ(v[0], 3.0f);
    EXPECT_FLOAT_EQ(v[1], 3.0f);
    EXPECT_FLOAT_EQ(v[2], 3.0f);
}

TEST_F(VectorTest, ScalarMultiplicationAssignment)
{
    Vec<float, 3> v = v3_a;
    v *= 2.0f;
    EXPECT_FLOAT_EQ(v[0], 2.0f);
    EXPECT_FLOAT_EQ(v[1], 4.0f);
    EXPECT_FLOAT_EQ(v[2], 6.0f);
}

TEST_F(VectorTest, ScalarDivisionAssignment)
{
    Vec<float, 3> v = v3_a;
    v /= 2.0f;
    EXPECT_FLOAT_EQ(v[0], 0.5f);
    EXPECT_FLOAT_EQ(v[1], 1.0f);
    EXPECT_FLOAT_EQ(v[2], 1.5f);
}

// ===== Dot Product Tests =====

TEST_F(VectorTest, DotProduct)
{
    float result = v3_a.dot(v3_b);
    EXPECT_FLOAT_EQ(result, 32.0f);  // 1*4 + 2*5 + 3*6 = 32
}

TEST_F(VectorTest, DotProductOperator)
{
    float result = v3_a | v3_b;
    EXPECT_FLOAT_EQ(result, 32.0f);
}

TEST_F(VectorTest, DotProductOrthogonal)
{
    float result = v3_unit_x.dot(v3_unit_y);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

// ===== Cross Product Tests =====

TEST_F(VectorTest, CrossProduct)
{
    Vec<float, 3> result = v3_unit_x.cross(v3_unit_y);
    EXPECT_NEAR(result[0], 0.0f, epsilon);
    EXPECT_NEAR(result[1], 0.0f, epsilon);
    EXPECT_NEAR(result[2], 1.0f, epsilon);
}

TEST_F(VectorTest, CrossProductOperator)
{
    Vec<float, 3> result = v3_unit_x ^ v3_unit_y;
    EXPECT_NEAR(result[0], 0.0f, epsilon);
    EXPECT_NEAR(result[1], 0.0f, epsilon);
    EXPECT_NEAR(result[2], 1.0f, epsilon);
}

TEST_F(VectorTest, CrossProductAnticommutative)
{
    Vec<float, 3> result1 = v3_a.cross(v3_b);
    Vec<float, 3> result2 = v3_b.cross(v3_a);
    EXPECT_NEAR(result1[0], -result2[0], epsilon);
    EXPECT_NEAR(result1[1], -result2[1], epsilon);
    EXPECT_NEAR(result1[2], -result2[2], epsilon);
}

// ===== Length and Normalization Tests =====

TEST_F(VectorTest, Length)
{
    float len = v2_a.length();
    EXPECT_FLOAT_EQ(len, 5.0f);  // sqrt(3^2 + 4^2) = 5
}

TEST_F(VectorTest, LengthSquared)
{
    float len_sq = v2_a.length_squared();
    EXPECT_FLOAT_EQ(len_sq, 25.0f);  // 3^2 + 4^2 = 25
}

TEST_F(VectorTest, Magnitude)
{
    float mag = v2_a.magnitude();
    EXPECT_FLOAT_EQ(mag, 5.0f);
}

TEST_F(VectorTest, MagnitudeSquared)
{
    float mag_sq = v2_a.magnitude_squared();
    EXPECT_FLOAT_EQ(mag_sq, 25.0f);
}

TEST_F(VectorTest, Normalized)
{
    Vec<float, 2> norm = v2_a.normalized();
    EXPECT_NEAR(norm[0], 0.6f, epsilon);
    EXPECT_NEAR(norm[1], 0.8f, epsilon);
    EXPECT_NEAR(norm.length(), 1.0f, epsilon);
}

TEST_F(VectorTest, NormalizedZeroVector)
{
    Vec<float, 3> norm = v3_zero.normalized();
    EXPECT_FLOAT_EQ(norm[0], 0.0f);
    EXPECT_FLOAT_EQ(norm[1], 0.0f);
    EXPECT_FLOAT_EQ(norm[2], 0.0f);
}

TEST_F(VectorTest, SafeNormalized)
{
    Vec<float, 3> norm = v3_zero.safe_normalized();
    EXPECT_FLOAT_EQ(norm[0], 0.0f);
    EXPECT_FLOAT_EQ(norm[1], 0.0f);
    EXPECT_FLOAT_EQ(norm[2], 0.0f);
}

// ===== Angle Tests =====

TEST_F(VectorTest, AngleBetweenVectors)
{
    float angle = v3_unit_x.angle(v3_unit_y);
    EXPECT_NEAR(angle, constants::pi<float>() / 2.0f, epsilon);
}

TEST_F(VectorTest, SignedAngle)
{
    float angle = v3_unit_x.signed_angle(v3_unit_y, v3_unit_z);
    EXPECT_NEAR(angle, constants::pi<float>() / 2.0f, epsilon);
}

// ===== Comparison Tests =====

TEST_F(VectorTest, Equality)
{
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(1.0f, 2.0f, 3.0f);
    EXPECT_TRUE(v1 == v2);
}

TEST_F(VectorTest, Inequality)
{
    EXPECT_TRUE(v3_a != v3_b);
}

TEST_F(VectorTest, ApproxEqual)
{
    Vec<float, 3> v1(1.0f, 2.0f, 3.0f);
    Vec<float, 3> v2(1.0f + 1e-7f, 2.0f, 3.0f);
    EXPECT_TRUE(v1.approx_equal(v2, 1e-6f));
}

// ===== Interpolation Tests =====

TEST_F(VectorTest, Lerp)
{
    Vec<float, 3> result = v3_a.lerp(v3_b, 0.5f);
    EXPECT_FLOAT_EQ(result[0], 2.5f);
    EXPECT_FLOAT_EQ(result[1], 3.5f);
    EXPECT_FLOAT_EQ(result[2], 4.5f);
}

TEST_F(VectorTest, LerpBoundaries)
{
    Vec<float, 3> result0 = v3_a.lerp(v3_b, 0.0f);
    Vec<float, 3> result1 = v3_a.lerp(v3_b, 1.0f);
    EXPECT_EQ(result0, v3_a);
    EXPECT_EQ(result1, v3_b);
}

// ===== Element-wise Operations =====

TEST_F(VectorTest, HadamardProduct)
{
    Vec<float, 3> result = v3_a.hadamard(v3_b);
    EXPECT_FLOAT_EQ(result[0], 4.0f);
    EXPECT_FLOAT_EQ(result[1], 10.0f);
    EXPECT_FLOAT_EQ(result[2], 18.0f);
}

TEST_F(VectorTest, ElementwiseMultiplication)
{
    Vec<float, 3> result = v3_a.elem_mult(v3_b);
    EXPECT_FLOAT_EQ(result[0], 4.0f);
    EXPECT_FLOAT_EQ(result[1], 10.0f);
    EXPECT_FLOAT_EQ(result[2], 18.0f);
}

TEST_F(VectorTest, MinMax)
{
    Vec<float, 3> v1(1.0f, 5.0f, 3.0f);
    Vec<float, 3> v2(2.0f, 4.0f, 6.0f);

    Vec<float, 3> vmin = v1.min(v2);
    Vec<float, 3> vmax = v1.max(v2);

    EXPECT_FLOAT_EQ(vmin[0], 1.0f);
    EXPECT_FLOAT_EQ(vmin[1], 4.0f);
    EXPECT_FLOAT_EQ(vmin[2], 3.0f);

    EXPECT_FLOAT_EQ(vmax[0], 2.0f);
    EXPECT_FLOAT_EQ(vmax[1], 5.0f);
    EXPECT_FLOAT_EQ(vmax[2], 6.0f);
}

TEST_F(VectorTest, MinMaxElement)
{
    Vec<float, 3> v(3.0f, 1.0f, 5.0f);
    EXPECT_FLOAT_EQ(v.min_element(), 1.0f);
    EXPECT_FLOAT_EQ(v.max_element(), 5.0f);
}

TEST_F(VectorTest, Abs)
{
    Vec<float, 3> v(-1.0f, 2.0f, -3.0f);
    Vec<float, 3> result = v.abs();
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
}

TEST_F(VectorTest, Clamp)
{
    Vec<float, 3> v(0.5f, 1.5f, 2.5f);
    Vec<float, 3> result = v.clamp(1.0f, 2.0f);
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 1.5f);
    EXPECT_FLOAT_EQ(result[2], 2.0f);
}

TEST_F(VectorTest, Saturate)
{
    Vec<float, 3> v(-0.5f, 0.5f, 1.5f);
    Vec<float, 3> result = v.saturated();
    EXPECT_FLOAT_EQ(result[0], 0.0f);
    EXPECT_FLOAT_EQ(result[1], 0.5f);
    EXPECT_FLOAT_EQ(result[2], 1.0f);
}

// ===== Projection Tests =====

TEST_F(VectorTest, Project)
{
    Vec<float, 3> v(1.0f, 1.0f, 0.0f);
    Vec<float, 3> onto(1.0f, 0.0f, 0.0f);
    Vec<float, 3> proj = v.project(onto);
    EXPECT_FLOAT_EQ(proj[0], 1.0f);
    EXPECT_FLOAT_EQ(proj[1], 0.0f);
    EXPECT_FLOAT_EQ(proj[2], 0.0f);
}

TEST_F(VectorTest, Reject)
{
    Vec<float, 3> v(1.0f, 1.0f, 0.0f);
    Vec<float, 3> from(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rej = v.reject(from);
    EXPECT_FLOAT_EQ(rej[0], 0.0f);
    EXPECT_FLOAT_EQ(rej[1], 1.0f);
    EXPECT_FLOAT_EQ(rej[2], 0.0f);
}

// ===== Distance Tests =====

TEST_F(VectorTest, Distance)
{
    Vec<float, 2> v1(0.0f, 0.0f);
    Vec<float, 2> v2(3.0f, 4.0f);
    float dist = v1.distance(v2);
    EXPECT_FLOAT_EQ(dist, 5.0f);
}

// ===== Swizzle Tests =====

TEST_F(VectorTest, SwizzleXY)
{
    Vec<float, 2> result = v3_a.xy();
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
}

TEST_F(VectorTest, SwizzleXYZ)
{
    Vec<float, 3> result = v4_a.xyz();
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
}

// ===== Homogeneous Coordinates =====

TEST_F(VectorTest, ToHomogeneous)
{
    Vec<float, 4> result = v3_a.to_homogeneous();
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
    EXPECT_FLOAT_EQ(result[3], 1.0f);
}

TEST_F(VectorTest, FromHomogeneous)
{
    Vec<float, 4> v(2.0f, 4.0f, 6.0f, 2.0f);
    Vec<float, 3> result = v.from_homogeneous();
    EXPECT_FLOAT_EQ(result[0], 1.0f);
    EXPECT_FLOAT_EQ(result[1], 2.0f);
    EXPECT_FLOAT_EQ(result[2], 3.0f);
}

// ===== Static Factory Methods =====

TEST_F(VectorTest, Zero)
{
    Vec<float, 3> v = Vec<float, 3>::zero();
    EXPECT_FLOAT_EQ(v[0], 0.0f);
    EXPECT_FLOAT_EQ(v[1], 0.0f);
    EXPECT_FLOAT_EQ(v[2], 0.0f);
}

TEST_F(VectorTest, One)
{
    Vec<float, 3> v = Vec<float, 3>::one();
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[1], 1.0f);
    EXPECT_FLOAT_EQ(v[2], 1.0f);
}

TEST_F(VectorTest, UnitVectors)
{
    Vec<float, 3> ux = Vec<float, 3>::unit_x();
    Vec<float, 3> uy = Vec<float, 3>::unit_y();
    Vec<float, 3> uz = Vec<float, 3>::unit_z();

    EXPECT_EQ(ux, v3_unit_x);
    EXPECT_EQ(uy, v3_unit_y);
    EXPECT_EQ(uz, v3_unit_z);
}

// ===== Rotation Tests =====

TEST_F(VectorTest, RotateAroundAxis)
{
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
    Vec<float, 3> rotated = v.rotate(axis, constants::pi<float>() / 2.0f);

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 1.0f, epsilon);
    EXPECT_NEAR(rotated[2], 0.0f, epsilon);
}

// ===== Type Tests =====

TEST(VectorTypeTest, IntegerVector)
{
    Vec<int, 3> v(1, 2, 3);
    EXPECT_EQ(v[0], 1);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 3);
}

TEST(VectorTypeTest, DoubleVector)
{
    Vec<double, 3> v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.0);
}

// ===== Edge Cases =====

TEST(VectorEdgeCaseTest, LargeVectors)
{
    Vec<float, 10> v;
    for (int i = 0; i < 10; ++i)
    {
        v[i] = static_cast<float>(i);
    }
    EXPECT_FLOAT_EQ(v[5], 5.0f);
}

TEST(VectorEdgeCaseTest, SingleElementVector)
{
    Vec<float, 1> v(42.0f);
    EXPECT_FLOAT_EQ(v[0], 42.0f);
}
