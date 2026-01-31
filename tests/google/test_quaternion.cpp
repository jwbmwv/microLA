// SPDX-License-Identifier: Apache-2.0
/// @file test_quaternion.cpp
/// @brief Comprehensive tests for Quaternion<T> class
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace microla;

class QuaternionTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        q_identity = Quaternion<float>::identity();
        q_a = Quaternion<float>(1.0F, 2.0F, 3.0F, 4.0F);
        q_b = Quaternion<float>(5.0F, 6.0F, 7.0F, 8.0F);

        axis_x = Vec<float, 3>(1.0F, 0.0F, 0.0F);
        axis_y = Vec<float, 3>(0.0F, 1.0F, 0.0F);
        axis_z = Vec<float, 3>(0.0F, 0.0F, 1.0F);
    }

    Quaternion<float> q_identity, q_a, q_b;
    Vec<float, 3> axis_x, axis_y, axis_z;
    const float epsilon = 1e-6F;
};

// ===== Construction Tests =====

TEST_F(QuaternionTest, DefaultConstructor)
{
    Quaternion<float> q;
    EXPECT_FLOAT_EQ(q.w(), 1.0F);
    EXPECT_FLOAT_EQ(q.x(), 0.0F);
    EXPECT_FLOAT_EQ(q.y(), 0.0F);
    EXPECT_FLOAT_EQ(q.z(), 0.0F);
}

TEST_F(QuaternionTest, ComponentConstructor)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    EXPECT_FLOAT_EQ(q.w(), 1.0F);
    EXPECT_FLOAT_EQ(q.x(), 2.0F);
    EXPECT_FLOAT_EQ(q.y(), 3.0F);
    EXPECT_FLOAT_EQ(q.z(), 4.0F);
}

TEST_F(QuaternionTest, AxisAngleConstructor)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    EXPECT_NEAR(q.w(), std::cos(constants::pi<float>() / 4.0F), epsilon);
    EXPECT_NEAR(q.z(), std::sin(constants::pi<float>() / 4.0F), epsilon);
}

TEST_F(QuaternionTest, CopyConstructor)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    const Quaternion<float>& q2(q1);
    EXPECT_EQ(q1, q2);
}

TEST_F(QuaternionTest, IdentityQuaternion)
{
    EXPECT_FLOAT_EQ(q_identity.w(), 1.0F);
    EXPECT_FLOAT_EQ(q_identity.x(), 0.0F);
    EXPECT_FLOAT_EQ(q_identity.y(), 0.0F);
    EXPECT_FLOAT_EQ(q_identity.z(), 0.0F);
    EXPECT_TRUE(q_identity.is_identity());
}

// ===== Accessors =====

TEST_F(QuaternionTest, ComponentAccess)
{
    EXPECT_FLOAT_EQ(q_a.w(), 1.0F);
    EXPECT_FLOAT_EQ(q_a.x(), 2.0F);
    EXPECT_FLOAT_EQ(q_a.y(), 3.0F);
    EXPECT_FLOAT_EQ(q_a.z(), 4.0F);
}

TEST_F(QuaternionTest, SubscriptOperator)
{
    // Data is stored as [X, Y, Z, W] internally
    EXPECT_FLOAT_EQ(q_a[0], 2.0F);  // X
    EXPECT_FLOAT_EQ(q_a[1], 3.0F);  // Y
    EXPECT_FLOAT_EQ(q_a[2], 4.0F);  // Z
    EXPECT_FLOAT_EQ(q_a[3], 1.0F);  // W
}

// ===== Arithmetic Operations =====

TEST_F(QuaternionTest, Addition)
{
    Quaternion<float> result = q_a + q_b;
    EXPECT_FLOAT_EQ(result.w(), 6.0F);
    EXPECT_FLOAT_EQ(result.x(), 8.0F);
    EXPECT_FLOAT_EQ(result.y(), 10.0F);
    EXPECT_FLOAT_EQ(result.z(), 12.0F);
}

TEST_F(QuaternionTest, Subtraction)
{
    Quaternion<float> result = q_b - q_a;
    EXPECT_FLOAT_EQ(result.w(), 4.0F);
    EXPECT_FLOAT_EQ(result.x(), 4.0F);
    EXPECT_FLOAT_EQ(result.y(), 4.0F);
    EXPECT_FLOAT_EQ(result.z(), 4.0F);
}

TEST_F(QuaternionTest, ScalarMultiplication)
{
    Quaternion<float> result = q_a * 2.0F;
    EXPECT_FLOAT_EQ(result.w(), 2.0F);
    EXPECT_FLOAT_EQ(result.x(), 4.0F);
    EXPECT_FLOAT_EQ(result.y(), 6.0F);
    EXPECT_FLOAT_EQ(result.z(), 8.0F);
}

TEST_F(QuaternionTest, QuaternionMultiplication)
{
    Quaternion<float> result = q_identity * q_a;
    EXPECT_EQ(result, q_a);
}

TEST_F(QuaternionTest, QuaternionMultiplicationNonCommutative)
{
    Quaternion<float> result1 = q_a * q_b;
    Quaternion<float> result2 = q_b * q_a;
    EXPECT_FALSE(result1 == result2);
}

// ===== Conjugate and Inverse =====

TEST_F(QuaternionTest, Conjugate)
{
    Quaternion<float> conj = q_a.conjugate();
    EXPECT_FLOAT_EQ(conj.w(), 1.0F);
    EXPECT_FLOAT_EQ(conj.x(), -2.0F);
    EXPECT_FLOAT_EQ(conj.y(), -3.0F);
    EXPECT_FLOAT_EQ(conj.z(), -4.0F);
}

TEST_F(QuaternionTest, Inverse)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    q = q.normalized();
    Quaternion<float> inv = q.inverse();
    Quaternion<float> product = q * inv;

    EXPECT_NEAR(product.w(), 1.0F, epsilon);
    EXPECT_NEAR(product.x(), 0.0F, epsilon);
    EXPECT_NEAR(product.y(), 0.0F, epsilon);
    EXPECT_NEAR(product.z(), 0.0F, epsilon);
}

TEST_F(QuaternionTest, InverseUnitMatchesInverse)
{
    Quaternion<float> q(axis_y, constants::pi<float>() / 3.0F);
    q = q.normalized();

    const Quaternion<float> inv = q.inverse();
    const Quaternion<float> inv_unit = q.inverse_unit();

    EXPECT_NEAR(inv_unit.w(), inv.w(), epsilon);
    EXPECT_NEAR(inv_unit.x(), inv.x(), epsilon);
    EXPECT_NEAR(inv_unit.y(), inv.y(), epsilon);
    EXPECT_NEAR(inv_unit.z(), inv.z(), epsilon);
}

TEST_F(QuaternionTest, IsIdentity)
{
    EXPECT_TRUE(q_identity.is_identity());
    EXPECT_TRUE(Quaternion<float>(1.0F, 2e-7F, -3e-7F, 1e-7F).is_identity());
    EXPECT_FALSE(q_a.is_identity());
}

// ===== Norm and Normalization =====

TEST_F(QuaternionTest, Norm)
{
    float n = q_a.norm();
    EXPECT_NEAR(n, std::sqrt(30.0F), epsilon);  // sqrt(1^2 + 2^2 + 3^2 + 4^2)
}

TEST_F(QuaternionTest, NormSquared)
{
    float n_sq = q_a.norm_squared();
    EXPECT_FLOAT_EQ(n_sq, 30.0F);
}

TEST_F(QuaternionTest, Normalized)
{
    Quaternion<float> norm = q_a.normalized();
    EXPECT_NEAR(norm.norm(), 1.0F, epsilon);
}

TEST_F(QuaternionTest, NormalizeInPlace)
{
    Quaternion<float> q = q_a;
    q.normalize();
    EXPECT_NEAR(q.norm(), 1.0F, epsilon);
}

// ===== Dot Product =====

TEST_F(QuaternionTest, DotProduct)
{
    float dot = q_a.dot(q_b);
    EXPECT_FLOAT_EQ(dot, 70.0F);  // 1*5 + 2*6 + 3*7 + 4*8
}

// ===== Rotation Operations =====

TEST_F(QuaternionTest, RotateVector)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q.rotate(v);

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

TEST_F(QuaternionTest, RotateVectorOperator)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

TEST_F(QuaternionTest, RotateInverseUndoesRotation)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    q = q.normalized();
    const Vec<float, 3> v(0.25F, -1.0F, 2.5F);

    const Vec<float, 3> rotated = q.rotate(v);
    const Vec<float, 3> restored = q.rotate_inverse(rotated);

    EXPECT_NEAR(restored[0], v[0], epsilon);
    EXPECT_NEAR(restored[1], v[1], epsilon);
    EXPECT_NEAR(restored[2], v[2], epsilon);
}

// ===== Conversion to Matrix =====

TEST_F(QuaternionTest, ToMatrix)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    Mat<float, 3, 3> R = q.to_matrix();
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

// ===== Euler Angles =====

TEST_F(QuaternionTest, ToEulerAngles)
{
    Quaternion<float> q(axis_z, constants::pi<float>() / 2.0F);
    Vec<float, 3> euler = q.to_euler();

    EXPECT_NEAR(euler[2], constants::pi<float>() / 2.0F, 0.01F);
}

TEST_F(QuaternionTest, FromEulerAngles)
{
    Vec<float, 3> euler(0.0F, 0.0F, constants::pi<float>() / 2.0F);
    Quaternion<float> q = Quaternion<float>::from_euler(euler);

    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q.rotate(v);

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
}

TEST_F(QuaternionTest, RollPitchYaw)
{
    Vec<float, 3> euler(0.1F, 0.2F, 0.3F);
    Quaternion<float> q = Quaternion<float>::from_euler(euler);

    EXPECT_NEAR(q.roll(), 0.1F, 0.01F);
    EXPECT_NEAR(q.pitch(), 0.2F, 0.01F);
    EXPECT_NEAR(q.yaw(), 0.3F, 0.01F);
}

// ===== SLERP =====

TEST_F(QuaternionTest, Slerp)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2(axis_z, constants::pi<float>() / 2.0F);

    Quaternion<float> q_half = q1.slerp(q2, 0.5F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q_half.rotate(v);

    // Should be roughly 45 degrees
    EXPECT_NEAR(rotated[0], rotated[1], epsilon);
}

TEST_F(QuaternionTest, SlerpBoundaries)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2(axis_z, constants::pi<float>() / 2.0F);

    Quaternion<float> q0 = q1.slerp(q2, 0.0F);
    Quaternion<float> q1_end = q1.slerp(q2, 1.0F);

    EXPECT_NEAR(q0.w(), q1.w(), epsilon);
    EXPECT_NEAR(q1_end.w(), q2.w(), epsilon);
}

// ===== Angle Between =====

TEST_F(QuaternionTest, AngleBetween)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2(axis_z, constants::pi<float>() / 2.0F);

    float angle = q1.angle_to(q2);
    EXPECT_NEAR(angle, constants::pi<float>() / 2.0F, 0.01F);
}

// ===== Rotation Composition =====

TEST_F(QuaternionTest, RotationComposition)
{
    Quaternion<float> q1(axis_z, constants::pi<float>() / 4.0F);
    Quaternion<float> q2(axis_z, constants::pi<float>() / 4.0F);
    Quaternion<float> q_combined = q2 * q1;

    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q_combined.rotate(v);

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
}

// ===== Static Factory Methods =====

TEST_F(QuaternionTest, FromAxisAngle)
{
    Quaternion<float> q = Quaternion<float>::from_axis_angle(axis_z, constants::pi<float>() / 2.0F);
    EXPECT_NEAR(q.w(), std::cos(constants::pi<float>() / 4.0F), epsilon);
}

TEST_F(QuaternionTest, FromTwoVectors)
{
    Vec<float, 3> from(1.0F, 0.0F, 0.0F);
    Vec<float, 3> to(0.0F, 1.0F, 0.0F);
    Quaternion<float> q = Quaternion<float>::from_two_vectors(from, to);

    Vec<float, 3> rotated = q.rotate(from);
    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
}

// ===== Comparison Tests =====

TEST_F(QuaternionTest, Equality)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(1.0F, 2.0F, 3.0F, 4.0F);
    EXPECT_TRUE(q1 == q2);
}

TEST_F(QuaternionTest, Inequality)
{
    EXPECT_TRUE(q_a != q_b);
}

TEST_F(QuaternionTest, ApproxEqual)
{
    Quaternion<float> q1(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> q2(1.0F + 1e-7F, 2.0F, 3.0F, 4.0F);
    EXPECT_TRUE(q1.approx_equal(q2, 1e-6F));
}

// ===== Type Tests =====

TEST(QuaternionTypeTest, DoubleQuaternion)
{
    Quaternion<double> q(1.0, 2.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 2.0);
}

// ===== Edge Cases =====

TEST(QuaternionEdgeCaseTest, ZeroNorm)
{
    Quaternion<float> q(0.0F, 0.0F, 0.0F, 0.0F);
    Quaternion<float> norm = q.normalized();
    // Should handle gracefully (return identity or zero)
    EXPECT_TRUE(std::isfinite(norm.w()));
}

TEST(QuaternionEdgeCaseTest, LargeAngles)
{
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    Quaternion<float> q(axis, 2.0F * constants::pi<float>());
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q.rotate(v);

    // 360 degrees should return to original
    EXPECT_NEAR(rotated[0], 1.0F, 0.01F);
    EXPECT_NEAR(rotated[1], 0.0F, 0.01F);
}

TEST(QuaternionEdgeCaseTest, OppositeVectors)
{
    Vec<float, 3> from(1.0F, 0.0F, 0.0F);
    Vec<float, 3> to(-1.0F, 0.0F, 0.0F);
    Quaternion<float> q = Quaternion<float>::from_two_vectors(from, to);

    Vec<float, 3> rotated = q.rotate(from);
    EXPECT_NEAR(rotated[0], -1.0F, 0.01F);
}

TEST(QuaternionEdgeCaseTest, SameVectors)
{
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Quaternion<float> q = Quaternion<float>::from_two_vectors(v, v);

    // Should be identity
    EXPECT_NEAR(q.w(), 1.0F, 0.01F);
    EXPECT_NEAR(q.x(), 0.0F, 0.01F);
    EXPECT_NEAR(q.y(), 0.0F, 0.01F);
    EXPECT_NEAR(q.z(), 0.0F, 0.01F);
}

// ===== Gimbal Lock Tests =====

TEST(QuaternionGimbalLockTest, NinetyDegreePitch)
{
    Vec<float, 3> euler(0.0F, constants::pi<float>() / 2.0F, 0.0F);
    Quaternion<float> q = Quaternion<float>::from_euler(euler);
    Vec<float, 3> euler_back = q.to_euler();

    // Should handle gimbal lock gracefully
    EXPECT_TRUE(std::isfinite(euler_back[0]));
    EXPECT_TRUE(std::isfinite(euler_back[1]));
    EXPECT_TRUE(std::isfinite(euler_back[2]));
}

// ===== Performance Tests =====

TEST(QuaternionPerformanceTest, MultipleRotations)
{
    Quaternion<float> q(Vec<float, 3>(0.0F, 0.0F, 1.0F), 0.01F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);

    // Apply 100 small rotations
    for (int i = 0; i < 100; ++i)
    {
        v = q.rotate(v);
    }

    // Should be roughly 1 radian total (57 degrees)
    EXPECT_TRUE(std::isfinite(v[0]));
    EXPECT_TRUE(std::isfinite(v[1]));
}
