// SPDX-License-Identifier: MIT
/// @file test_matrix.cpp
/// @brief Comprehensive tests for Mat<T,R,C> class
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace microla;

class MatrixTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mat2_identity = Mat<float, 2, 2>::identity();
        mat3_identity = Mat<float, 3, 3>::identity();
        mat4_identity = Mat<float, 4, 4>::identity();

        mat2_a = Mat<float, 2, 2>({1.0f, 2.0f, 3.0f, 4.0f});
        mat2_b = Mat<float, 2, 2>({5.0f, 6.0f, 7.0f, 8.0f});

        mat3_a = Mat<float, 3, 3>({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f});
    }

    Mat<float, 2, 2> mat2_identity, mat2_a, mat2_b;
    Mat<float, 3, 3> mat3_identity, mat3_a;
    Mat<float, 4, 4> mat4_identity;
    const float epsilon = 1e-6f;
};

// ===== Construction Tests =====

TEST_F(MatrixTest, DefaultConstructor)
{
    Mat<float, 2, 2> m;
    EXPECT_FLOAT_EQ(m(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 0.0f);
}

TEST_F(MatrixTest, InitializerListConstructor)
{
    Mat<float, 2, 2> m({1.0f, 2.0f, 3.0f, 4.0f});
    EXPECT_FLOAT_EQ(m(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(m(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0f);
}

TEST_F(MatrixTest, CopyConstructor)
{
    Mat<float, 2, 2> m1({1.0f, 2.0f, 3.0f, 4.0f});
    Mat<float, 2, 2> m2(m1);
    EXPECT_EQ(m1, m2);
}

TEST_F(MatrixTest, IdentityMatrix)
{
    EXPECT_FLOAT_EQ(mat2_identity(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(mat2_identity(0, 1), 0.0f);
    EXPECT_FLOAT_EQ(mat2_identity(1, 0), 0.0f);
    EXPECT_FLOAT_EQ(mat2_identity(1, 1), 1.0f);
}

// ===== Accessors =====

TEST_F(MatrixTest, ParenthesisOperator)
{
    EXPECT_FLOAT_EQ(mat2_a(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(mat2_a(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(mat2_a(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(mat2_a(1, 1), 4.0f);
}

TEST_F(MatrixTest, RowAccess)
{
    EXPECT_FLOAT_EQ(mat2_a[0][0], 1.0f);
    EXPECT_FLOAT_EQ(mat2_a[0][1], 2.0f);
}

// ===== Arithmetic Operations =====

TEST_F(MatrixTest, Addition)
{
    Mat<float, 2, 2> result = mat2_a + mat2_b;
    EXPECT_FLOAT_EQ(result(0, 0), 6.0f);
    EXPECT_FLOAT_EQ(result(0, 1), 8.0f);
    EXPECT_FLOAT_EQ(result(1, 0), 10.0f);
    EXPECT_FLOAT_EQ(result(1, 1), 12.0f);
}

TEST_F(MatrixTest, Subtraction)
{
    Mat<float, 2, 2> result = mat2_b - mat2_a;
    EXPECT_FLOAT_EQ(result(0, 0), 4.0f);
    EXPECT_FLOAT_EQ(result(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(result(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(result(1, 1), 4.0f);
}

TEST_F(MatrixTest, ScalarMultiplication)
{
    Mat<float, 2, 2> result = mat2_a * 2.0f;
    EXPECT_FLOAT_EQ(result(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(result(0, 1), 4.0f);
    EXPECT_FLOAT_EQ(result(1, 0), 6.0f);
    EXPECT_FLOAT_EQ(result(1, 1), 8.0f);
}

TEST_F(MatrixTest, ScalarDivision)
{
    Mat<float, 2, 2> result = mat2_a / 2.0f;
    EXPECT_FLOAT_EQ(result(0, 0), 0.5f);
    EXPECT_FLOAT_EQ(result(0, 1), 1.0f);
    EXPECT_FLOAT_EQ(result(1, 0), 1.5f);
    EXPECT_FLOAT_EQ(result(1, 1), 2.0f);
}

TEST_F(MatrixTest, MatrixMultiplication)
{
    Mat<float, 2, 2> result = mat2_a * mat2_identity;
    EXPECT_EQ(result, mat2_a);
}

TEST_F(MatrixTest, MatrixVectorMultiplication)
{
    Vec<float, 2> v(1.0f, 2.0f);
    Vec<float, 2> result = mat2_a * v;
    EXPECT_FLOAT_EQ(result[0], 5.0f);   // 1*1 + 2*2
    EXPECT_FLOAT_EQ(result[1], 11.0f);  // 3*1 + 4*2
}

// ===== Transpose =====

TEST_F(MatrixTest, Transpose)
{
    Mat<float, 2, 2> result = mat2_a.transpose();
    EXPECT_FLOAT_EQ(result(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(result(0, 1), 3.0f);
    EXPECT_FLOAT_EQ(result(1, 0), 2.0f);
    EXPECT_FLOAT_EQ(result(1, 1), 4.0f);
}

// ===== Determinant =====

TEST_F(MatrixTest, Determinant2x2)
{
    float det = mat2_a.determinant();
    EXPECT_FLOAT_EQ(det, -2.0f);  // 1*4 - 2*3 = -2
}

TEST_F(MatrixTest, Determinant3x3)
{
    Mat<float, 3, 3> m({1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f});
    float det = m.determinant();
    EXPECT_FLOAT_EQ(det, 1.0f);
}

// ===== Trace =====

TEST_F(MatrixTest, Trace)
{
    float tr = mat2_a.trace();
    EXPECT_FLOAT_EQ(tr, 5.0f);  // 1 + 4
}

// ===== Inverse =====

TEST_F(MatrixTest, InverseIdentity)
{
    Mat<float, 2, 2> inv = mat2_identity.inverse();
    EXPECT_EQ(inv, mat2_identity);
}

TEST_F(MatrixTest, Inverse2x2)
{
    Mat<float, 2, 2> m({4.0f, 7.0f, 2.0f, 6.0f});
    Mat<float, 2, 2> inv = m.inverse();
    Mat<float, 2, 2> product = m * inv;

    EXPECT_NEAR(product(0, 0), 1.0f, epsilon);
    EXPECT_NEAR(product(0, 1), 0.0f, epsilon);
    EXPECT_NEAR(product(1, 0), 0.0f, epsilon);
    EXPECT_NEAR(product(1, 1), 1.0f, epsilon);
}

// ===== Rotation Matrices =====

TEST_F(MatrixTest, RotationX)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_x(constants::pi<float>() / 2.0f);
    Vec<float, 3> v(0.0f, 1.0f, 0.0f);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 0.0f, epsilon);
    EXPECT_NEAR(rotated[2], 1.0f, epsilon);
}

TEST_F(MatrixTest, RotationY)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_y(constants::pi<float>() / 2.0f);
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 0.0f, epsilon);
    EXPECT_NEAR(rotated[2], -1.0f, epsilon);
}

TEST_F(MatrixTest, RotationZ)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 2.0f);
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 1.0f, epsilon);
    EXPECT_NEAR(rotated[2], 0.0f, epsilon);
}

TEST_F(MatrixTest, Rotation2D)
{
    Mat<float, 2, 2> R = Mat<float, 2, 2>::rotation(constants::pi<float>() / 2.0f);
    Vec<float, 2> v(1.0f, 0.0f);
    Vec<float, 2> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 1.0f, epsilon);
}

TEST_F(MatrixTest, RotationAxisAngle)
{
    Vec<float, 3> axis(0.0f, 0.0f, 1.0f);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_axis_angle(axis, constants::pi<float>() / 2.0f);
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 1.0f, epsilon);
    EXPECT_NEAR(rotated[2], 0.0f, epsilon);
}

TEST_F(MatrixTest, RotationFromTo)
{
    Vec<float, 3> from(1.0f, 0.0f, 0.0f);
    Vec<float, 3> to(0.0f, 1.0f, 0.0f);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_from_to(from, to);
    Vec<float, 3> rotated = R * from;

    EXPECT_NEAR(rotated[0], 0.0f, epsilon);
    EXPECT_NEAR(rotated[1], 1.0f, epsilon);
    EXPECT_NEAR(rotated[2], 0.0f, epsilon);
}

TEST_F(MatrixTest, LookAt)
{
    Vec<float, 3> target(1.0f, 0.0f, 0.0f);
    Vec<float, 3> up(0.0f, 1.0f, 0.0f);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::look_at(target, up);

    // Forward should point to target
    Vec<float, 3> forward(R(0, 0), R(1, 0), R(2, 0));
    EXPECT_NEAR(forward.length(), 1.0f, epsilon);
}

// ===== Euler Angles =====

TEST_F(MatrixTest, EulerAngles)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 4.0f);
    Vec<float, 3> euler = R.euler_angles();

    EXPECT_NEAR(euler[2], constants::pi<float>() / 4.0f, 0.01f);
}

// ===== Block Operations =====

TEST_F(MatrixTest, BlockExtraction)
{
    Mat<float, 3, 3> m({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f});
    Mat<float, 2, 2> block = m.block<2, 2>(0, 0);

    EXPECT_FLOAT_EQ(block(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(block(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(block(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(block(1, 1), 5.0f);
}

TEST_F(MatrixTest, SetBlock)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::zero();
    Mat<float, 2, 2> block({1.0f, 2.0f, 3.0f, 4.0f});
    m.set_block(0, 0, block);

    EXPECT_FLOAT_EQ(m(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(m(1, 0), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0f);
}

TEST_F(MatrixTest, RowExtraction)
{
    Mat<float, 1, 3> row = mat3_a.row(0);
    EXPECT_FLOAT_EQ(row(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(row(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(row(0, 2), 3.0f);
}

TEST_F(MatrixTest, ColumnExtraction)
{
    Mat<float, 3, 1> col = mat3_a.col(0);
    EXPECT_FLOAT_EQ(col(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(col(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(col(2, 0), 7.0f);
}

// ===== Decompositions =====

TEST_F(MatrixTest, LUDecomposition)
{
    Mat<float, 3, 3> m({2.0f, -1.0f, 0.0f, -1.0f, 2.0f, -1.0f, 0.0f, -1.0f, 2.0f});
#if __cplusplus >= 201703L
    auto [L, U, P] = m.lu();
#else
    auto lu_result = m.lu();
    auto L = std::get<0>(lu_result);
    auto U = std::get<1>(lu_result);
    auto P = std::get<2>(lu_result);
#endif

    Mat<float, 3, 3> reconstructed = P * L * U;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), epsilon);
        }
    }
}

TEST_F(MatrixTest, QRDecomposition)
{
    Mat<float, 3, 3> m({12.0f, -51.0f, 4.0f, 6.0f, 167.0f, -68.0f, -4.0f, 24.0f, -41.0f});
#if __cplusplus >= 201703L
    auto [Q, R] = m.qr();
#else
    auto qr_result = m.qr();
    auto Q = std::get<0>(qr_result);
    auto R = std::get<1>(qr_result);
#endif

    Mat<float, 3, 3> reconstructed = Q * R;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), 0.01f);
        }
    }
}

TEST_F(MatrixTest, CholeskyDecomposition)
{
    Mat<float, 3, 3> m({4.0f, 12.0f, -16.0f, 12.0f, 37.0f, -43.0f, -16.0f, -43.0f, 98.0f});
    Mat<float, 3, 3> L = m.cholesky();
    Mat<float, 3, 3> reconstructed = L * L.transpose();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), epsilon);
        }
    }
}

TEST_F(MatrixTest, SVDDecomposition)
{
    Mat<float, 3, 3> m({3.0f, 2.0f, 2.0f, 2.0f, 3.0f, -2.0f, 2.0f, -2.0f, 3.0f});
    auto result = m.svd();
    auto& U = std::get<0>(result);
    auto& S = std::get<1>(result);
    auto& V = std::get<2>(result);

    Mat<float, 3, 3> reconstructed = U * S * V.transpose();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), 0.01f);
        }
    }
}

// ===== Eigenvalues =====

TEST_F(MatrixTest, Eigenvalues)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::identity();
    std::vector<float> eigenvalues = m.eigenvaluesQR();

    EXPECT_EQ(eigenvalues.size(), 3);
    for (float ev : eigenvalues)
    {
        EXPECT_NEAR(ev, 1.0f, 0.01f);
    }
}

// ===== Norms =====

TEST_F(MatrixTest, FrobeniusNorm)
{
    Mat<float, 2, 2> m({1.0f, 2.0f, 3.0f, 4.0f});
    float norm = m.frobenius_norm();
    EXPECT_NEAR(norm, std::sqrt(30.0f), epsilon);
}

TEST_F(MatrixTest, InfinityNorm)
{
    Mat<float, 2, 2> m({1.0f, 2.0f, 3.0f, 4.0f});
    float norm = m.infinity_norm();
    EXPECT_FLOAT_EQ(norm, 7.0f);  // max row sum: |3| + |4| = 7
}

TEST_F(MatrixTest, OneNorm)
{
    Mat<float, 2, 2> m({1.0f, 2.0f, 3.0f, 4.0f});
    float norm = m.one_norm();
    EXPECT_FLOAT_EQ(norm, 6.0f);  // max col sum: |2| + |4| = 6
}

TEST_F(MatrixTest, ConditionNumber)
{
    Mat<float, 2, 2> m({1.0f, 0.0f, 0.0f, 1.0f});
    float cond = m.condition_number();
    EXPECT_NEAR(cond, 1.0f, epsilon);
}

// ===== Pseudoinverse =====

TEST_F(MatrixTest, Pseudoinverse)
{
    Mat<float, 2, 3> m({1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f});
    Mat<float, 3, 2> pinv = m.pseudoinverse();

    Mat<float, 2, 2> product = m * pinv;
    EXPECT_NEAR(product(0, 0), 1.0f, 0.01f);
    EXPECT_NEAR(product(1, 1), 1.0f, 0.01f);
}

// ===== Static Factory Methods =====

TEST_F(MatrixTest, Zero)
{
    Mat<float, 2, 2> m = Mat<float, 2, 2>::zero();
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            EXPECT_FLOAT_EQ(m(i, j), 0.0f);
        }
    }
}

TEST_F(MatrixTest, Ones)
{
    Mat<float, 2, 2> m = Mat<float, 2, 2>::ones();
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            EXPECT_FLOAT_EQ(m(i, j), 1.0f);
        }
    }
}

TEST_F(MatrixTest, Diagonal)
{
    Vec<float, 3> diag(1.0f, 2.0f, 3.0f);
    Mat<float, 3, 3> m = Mat<float, 3, 3>::diagonal(diag);

    EXPECT_FLOAT_EQ(m(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(m(1, 1), 2.0f);
    EXPECT_FLOAT_EQ(m(2, 2), 3.0f);
    EXPECT_FLOAT_EQ(m(0, 1), 0.0f);
}

// ===== Comparison Tests =====

TEST_F(MatrixTest, Equality)
{
    Mat<float, 2, 2> m1({1.0f, 2.0f, 3.0f, 4.0f});
    Mat<float, 2, 2> m2({1.0f, 2.0f, 3.0f, 4.0f});
    EXPECT_TRUE(m1 == m2);
}

TEST_F(MatrixTest, Inequality)
{
    EXPECT_TRUE(mat2_a != mat2_b);
}

// ===== Type Tests =====

TEST(MatrixTypeTest, IntegerMatrix)
{
    Mat<int, 2, 2> m({1, 2, 3, 4});
    EXPECT_EQ(m(0, 0), 1);
    EXPECT_EQ(m(1, 1), 4);
}

TEST(MatrixTypeTest, DoubleMatrix)
{
    Mat<double, 2, 2> m({1.0, 2.0, 3.0, 4.0});
    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
}

// ===== Non-square Matrices =====

TEST(MatrixNonSquareTest, RectangularMatrix)
{
    Mat<float, 2, 3> m({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    EXPECT_FLOAT_EQ(m(0, 2), 3.0f);
    EXPECT_FLOAT_EQ(m(1, 2), 6.0f);
}

TEST(MatrixNonSquareTest, RectangularMultiplication)
{
    Mat<float, 2, 3> A({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    Mat<float, 3, 2> B({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    Mat<float, 2, 2> C = A * B;

    EXPECT_FLOAT_EQ(C(0, 0), 22.0f);
    EXPECT_FLOAT_EQ(C(0, 1), 28.0f);
}

// ===== Edge Cases =====

TEST(MatrixEdgeCaseTest, 1x1Matrix)
{
    Mat<float, 1, 1> m({42.0f});
    EXPECT_FLOAT_EQ(m(0, 0), 42.0f);
    EXPECT_FLOAT_EQ(m.determinant(), 42.0f);
}

TEST(MatrixEdgeCaseTest, LargeMatrix)
{
    Mat<float, 10, 10> m = Mat<float, 10, 10>::identity();
    EXPECT_FLOAT_EQ(m(5, 5), 1.0f);
    EXPECT_FLOAT_EQ(m(5, 6), 0.0f);
}
