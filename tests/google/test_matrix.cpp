// SPDX-License-Identifier: Apache-2.0
/// @file test_matrix.cpp
/// @brief Comprehensive tests for Mat<T,R,C> class
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

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

        // clang-format off
        mat2_a = Mat<float, 2, 2>({
            {1.0F, 2.0F},
            {3.0F, 4.0F}
        });
        mat2_b = Mat<float, 2, 2>({
            {5.0F, 6.0F},
            {7.0F, 8.0F}
        });

        mat3_a = Mat<float, 3, 3>({
            {1.0F, 2.0F, 3.0F},
            {4.0F, 5.0F, 6.0F},
            {7.0F, 8.0F, 9.0F}
        });
        // clang-format on
    }

    Mat<float, 2, 2> mat2_identity, mat2_a, mat2_b;
    Mat<float, 3, 3> mat3_identity, mat3_a;
    Mat<float, 4, 4> mat4_identity;
    const float epsilon = 1e-6F;
};

// ===== Construction Tests =====

TEST_F(MatrixTest, DefaultConstructor)
{
    Mat<float, 2, 2> m;
    EXPECT_FLOAT_EQ(m(0, 0), 0.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 0.0F);
}

TEST_F(MatrixTest, InitializerListConstructor)
{
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 2.0F},
        {3.0F, 4.0F}
    });
    // clang-format on
    EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(m(1, 0), 3.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0F);
}

TEST_F(MatrixTest, CopyConstructor)
{
    // clang-format off
    Mat<float, 2, 2> m1({
        {1.0F, 2.0F},
        {3.0F, 4.0F}
    });
    // clang-format on
    Mat<float, 2, 2> m2(m1);
    EXPECT_EQ(m1, m2);
}

TEST_F(MatrixTest, IdentityMatrix)
{
    EXPECT_FLOAT_EQ(mat2_identity(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(mat2_identity(0, 1), 0.0F);
    EXPECT_FLOAT_EQ(mat2_identity(1, 0), 0.0F);
    EXPECT_FLOAT_EQ(mat2_identity(1, 1), 1.0F);
}

// ===== Accessors =====

TEST_F(MatrixTest, ParenthesisOperator)
{
    EXPECT_FLOAT_EQ(mat2_a(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(mat2_a(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(mat2_a(1, 0), 3.0F);
    EXPECT_FLOAT_EQ(mat2_a(1, 1), 4.0F);
}

TEST_F(MatrixTest, RowAccess)
{
    EXPECT_FLOAT_EQ(mat2_a[0][0], 1.0F);
    EXPECT_FLOAT_EQ(mat2_a[0][1], 2.0F);
}

// ===== Arithmetic Operations =====

TEST_F(MatrixTest, Addition)
{
    Mat<float, 2, 2> result = mat2_a + mat2_b;
    EXPECT_FLOAT_EQ(result(0, 0), 6.0F);
    EXPECT_FLOAT_EQ(result(0, 1), 8.0F);
    EXPECT_FLOAT_EQ(result(1, 0), 10.0F);
    EXPECT_FLOAT_EQ(result(1, 1), 12.0F);
}

TEST_F(MatrixTest, Subtraction)
{
    Mat<float, 2, 2> result = mat2_b - mat2_a;
    EXPECT_FLOAT_EQ(result(0, 0), 4.0F);
    EXPECT_FLOAT_EQ(result(0, 1), 4.0F);
    EXPECT_FLOAT_EQ(result(1, 0), 4.0F);
    EXPECT_FLOAT_EQ(result(1, 1), 4.0F);
}

TEST_F(MatrixTest, ScalarMultiplication)
{
    Mat<float, 2, 2> result = mat2_a * 2.0F;
    EXPECT_FLOAT_EQ(result(0, 0), 2.0F);
    EXPECT_FLOAT_EQ(result(0, 1), 4.0F);
    EXPECT_FLOAT_EQ(result(1, 0), 6.0F);
    EXPECT_FLOAT_EQ(result(1, 1), 8.0F);
}

TEST_F(MatrixTest, ScalarDivision)
{
    Mat<float, 2, 2> result = mat2_a / 2.0F;
    EXPECT_FLOAT_EQ(result(0, 0), 0.5F);
    EXPECT_FLOAT_EQ(result(0, 1), 1.0F);
    EXPECT_FLOAT_EQ(result(1, 0), 1.5F);
    EXPECT_FLOAT_EQ(result(1, 1), 2.0F);
}

TEST_F(MatrixTest, MatrixMultiplication)
{
    Mat<float, 2, 2> result = mat2_a * mat2_identity;
    EXPECT_EQ(result, mat2_a);
}

TEST_F(MatrixTest, MatrixVectorMultiplication)
{
    Vec<float, 2> v(1.0F, 2.0F);
    Vec<float, 2> result = mat2_a * v;
    EXPECT_FLOAT_EQ(result[0], 5.0F);   // 1*1 + 2*2
    EXPECT_FLOAT_EQ(result[1], 11.0F);  // 3*1 + 4*2
}

// ===== Transpose =====

TEST_F(MatrixTest, Transpose)
{
    Mat<float, 2, 2> result = mat2_a.transpose();
    EXPECT_FLOAT_EQ(result(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(result(0, 1), 3.0F);
    EXPECT_FLOAT_EQ(result(1, 0), 2.0F);
    EXPECT_FLOAT_EQ(result(1, 1), 4.0F);
}

// ===== Determinant =====

TEST_F(MatrixTest, Determinant2x2)
{
    float det = mat2_a.determinant();
    EXPECT_FLOAT_EQ(det, -2.0F);  // 1*4 - 2*3 = -2
}

TEST_F(MatrixTest, Determinant3x3)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F}
    });
    // clang-format on
    float det = m.determinant();
    EXPECT_FLOAT_EQ(det, 1.0F);
}

// ===== Trace =====

TEST_F(MatrixTest, Trace)
{
    float tr = mat2_a.trace();
    EXPECT_FLOAT_EQ(tr, 5.0F);  // 1 + 4
}

// ===== Inverse =====

TEST_F(MatrixTest, InverseIdentity)
{
    Mat<float, 2, 2> inv = mat2_identity.inverse();
    EXPECT_EQ(inv, mat2_identity);
}

TEST_F(MatrixTest, Inverse2x2)
{
    Mat<float, 2, 2> m({4.0F, 7.0F, 2.0F, 6.0F});
    Mat<float, 2, 2> inv = m.inverse();
    Mat<float, 2, 2> product = m * inv;

    EXPECT_NEAR(product(0, 0), 1.0F, epsilon);
    EXPECT_NEAR(product(0, 1), 0.0F, epsilon);
    EXPECT_NEAR(product(1, 0), 0.0F, epsilon);
    EXPECT_NEAR(product(1, 1), 1.0F, epsilon);
}

// ===== Rotation Matrices =====

TEST_F(MatrixTest, RotationX)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_x(constants::pi<float>() / 2.0F);
    Vec<float, 3> v(0.0F, 1.0F, 0.0F);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 0.0F, epsilon);
    EXPECT_NEAR(rotated[2], 1.0F, epsilon);
}

TEST_F(MatrixTest, RotationY)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_y(constants::pi<float>() / 2.0F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 0.0F, epsilon);
    EXPECT_NEAR(rotated[2], -1.0F, epsilon);
}

TEST_F(MatrixTest, RotationZ)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 2.0F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

TEST_F(MatrixTest, Rotation2D)
{
    Mat<float, 2, 2> R = Mat<float, 2, 2>::rotation(constants::pi<float>() / 2.0F);
    Vec<float, 2> v(1.0F, 0.0F);
    Vec<float, 2> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
}

TEST_F(MatrixTest, RotationAxisAngle)
{
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_axis_angle(axis, constants::pi<float>() / 2.0F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = R * v;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

TEST_F(MatrixTest, RotationFromTo)
{
    Vec<float, 3> from(1.0F, 0.0F, 0.0F);
    Vec<float, 3> to(0.0F, 1.0F, 0.0F);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_from_to(from, to);
    Vec<float, 3> rotated = R * from;

    EXPECT_NEAR(rotated[0], 0.0F, epsilon);
    EXPECT_NEAR(rotated[1], 1.0F, epsilon);
    EXPECT_NEAR(rotated[2], 0.0F, epsilon);
}

TEST_F(MatrixTest, LookAt)
{
    Vec<float, 3> target(1.0F, 0.0F, 0.0F);
    Vec<float, 3> up(0.0F, 1.0F, 0.0F);
    Mat<float, 3, 3> R = Mat<float, 3, 3>::look_at(target, up);

    // Forward should point to target
    Vec<float, 3> forward(R(0, 0), R(1, 0), R(2, 0));
    EXPECT_NEAR(forward.length(), 1.0F, epsilon);
}

// ===== Euler Angles =====

TEST_F(MatrixTest, EulerAngles)
{
    Mat<float, 3, 3> R = Mat<float, 3, 3>::rotation_z(constants::pi<float>() / 4.0F);
    Vec<float, 3> euler = R.euler_angles();

    EXPECT_NEAR(euler[2], constants::pi<float>() / 4.0F, 0.01F);
}

// ===== Block Operations =====

TEST_F(MatrixTest, BlockExtraction)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F}
    });
    // clang-format on
    Mat<float, 2, 2> block = m.block<2, 2>(0, 0);

    EXPECT_FLOAT_EQ(block(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(block(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(block(1, 0), 4.0F);
    EXPECT_FLOAT_EQ(block(1, 1), 5.0F);
}

TEST_F(MatrixTest, SetBlock)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::zero();
    // clang-format off
    Mat<float, 2, 2> block({
        {1.0F, 2.0F},
        {3.0F, 4.0F}
    });
    // clang-format on
    m.set_block(0, 0, block);

    EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(m(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(m(1, 0), 3.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 4.0F);
}

TEST_F(MatrixTest, RowExtraction)
{
    Mat<float, 1, 3> row = mat3_a.row(0);
    EXPECT_FLOAT_EQ(row(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(row(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(row(0, 2), 3.0F);
}

TEST_F(MatrixTest, ColumnExtraction)
{
    Mat<float, 3, 1> col = mat3_a.col(0);
    EXPECT_FLOAT_EQ(col(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(col(1, 0), 4.0F);
    EXPECT_FLOAT_EQ(col(2, 0), 7.0F);
}

// ===== Decompositions =====

TEST_F(MatrixTest, LUDecomposition)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {2.0F, -1.0F, 0.0F},
        {-1.0F, 2.0F, -1.0F},
        {0.0F, -1.0F, 2.0F}
    });
    // clang-format on
    auto [L, U, P] = m.lu();

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
    // clang-format off
    Mat<float, 3, 3> m({
        {12.0F, -51.0F, 4.0F},
        {6.0F, 167.0F, -68.0F},
        {-4.0F, 24.0F, -41.0F}
    });
    // clang-format on
    auto [Q, R] = m.qr();

    Mat<float, 3, 3> reconstructed = Q * R;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), 0.01F);
        }
    }
}

TEST_F(MatrixTest, CholeskyDecomposition)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {4.0F, 12.0F, -16.0F},
        {12.0F, 37.0F, -43.0F},
        {-16.0F, -43.0F, 98.0F}
    });
    // clang-format on
    auto l_result = m.cholesky();
    ASSERT_TRUE(l_result.has_value());
    Mat<float, 3, 3> L = *l_result;
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
    // clang-format off
    Mat<float, 3, 3> m({
        {3.0F, 2.0F, 2.0F},
        {2.0F, 3.0F, -2.0F},
        {2.0F, -2.0F, 3.0F}
    });
    // clang-format on
    auto result = m.svd();
    auto& U = std::get<0>(result);
    auto& S = std::get<1>(result);
    auto& V = std::get<2>(result);

    Mat<float, 3, 3> reconstructed = U * S * V.transpose();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), m(i, j), 0.01F);
        }
    }
}

// ===== Eigenvalues =====

TEST_F(MatrixTest, EigenvaluesBufferOverload)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::identity();
    float eigenvalues[3] = {};

    ASSERT_TRUE(m.eigenvalues_qr(eigenvalues, 3));
    for (float ev : eigenvalues)
    {
        EXPECT_NEAR(ev, 1.0F, 0.01F);
    }
}

#if MICROLA_HAS_DYNAMIC_ALLOC
TEST_F(MatrixTest, Eigenvalues)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::identity();
    auto eigenvalues_result = m.eigenvalues_qr();
    ASSERT_TRUE(std::holds_alternative<std::vector<float>>(eigenvalues_result));
    const auto& eigenvalues = std::get<std::vector<float>>(eigenvalues_result);

    EXPECT_EQ(eigenvalues.size(), 3);
    for (float ev : eigenvalues)
    {
        EXPECT_NEAR(ev, 1.0F, 0.01F);
    }
}
#endif

// ===== Norms =====

TEST_F(MatrixTest, FrobeniusNorm)
{
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 2.0F},
        {3.0F, 4.0F}
    });
    // clang-format on
    float norm = m.frobenius_norm();
    EXPECT_NEAR(norm, std::sqrt(30.0F), epsilon);
}

TEST_F(MatrixTest, InfinityNorm)
{
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 2.0F},
        {3.0F, 4.0F}
    });
    // clang-format on
    float norm = m.infinity_norm();
    EXPECT_FLOAT_EQ(norm, 7.0F);  // max row sum: |3| + |4| = 7
}

TEST_F(MatrixTest, OneNorm)
{
    Mat<float, 2, 2> m({1.0F, 2.0F, 3.0F, 4.0F});
    float norm = m.one_norm();
    EXPECT_FLOAT_EQ(norm, 6.0F);  // max col sum: |2| + |4| = 6
}

TEST_F(MatrixTest, ConditionNumber)
{
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 0.0F},
        {0.0F, 1.0F}
    });
    // clang-format on
    float cond = m.condition_number();
    EXPECT_NEAR(cond, 1.0F, epsilon);
}

// ===== Pseudoinverse =====

TEST_F(MatrixTest, Pseudoinverse)
{
    // clang-format off
    Mat<float, 2, 3> m({
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F}
    });
    // clang-format on
    Mat<float, 3, 2> pinv = m.pseudoinverse();

    Mat<float, 2, 2> product = m * pinv;
    EXPECT_NEAR(product(0, 0), 1.0F, 0.01F);
    EXPECT_NEAR(product(1, 1), 1.0F, 0.01F);
}

// ===== Static Factory Methods =====

TEST_F(MatrixTest, Zero)
{
    Mat<float, 2, 2> m = Mat<float, 2, 2>::zero();
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            EXPECT_FLOAT_EQ(m(i, j), 0.0F);
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
            EXPECT_FLOAT_EQ(m(i, j), 1.0F);
        }
    }
}

TEST_F(MatrixTest, Diagonal)
{
    Vec<float, 3> diag(1.0F, 2.0F, 3.0F);
    Mat<float, 3, 3> m = Mat<float, 3, 3>::diagonal(diag);

    EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 2.0F);
    EXPECT_FLOAT_EQ(m(2, 2), 3.0F);
    EXPECT_FLOAT_EQ(m(0, 1), 0.0F);
}

// ===== Comparison Tests =====

TEST_F(MatrixTest, Equality)
{
    Mat<float, 2, 2> m1({1.0F, 2.0F, 3.0F, 4.0F});
    Mat<float, 2, 2> m2({1.0F, 2.0F, 3.0F, 4.0F});
    EXPECT_TRUE(m1 == m2);
}

TEST_F(MatrixTest, Inequality)
{
    EXPECT_TRUE(mat2_a != mat2_b);
}

TEST_F(MatrixTest, IsIdentity)
{
    Mat<float, 3, 3> exact_identity = Mat<float, 3, 3>::identity();
    Mat<float, 3, 3> nearly_identity = Mat<float, 3, 3>::identity();
    Mat<float, 3, 3> not_identity = Mat<float, 3, 3>::identity();

    nearly_identity(0, 1) = 2e-7F;
    not_identity(1, 2) = 1e-3F;

    EXPECT_TRUE(exact_identity.is_identity());
    EXPECT_TRUE(nearly_identity.is_identity());
    EXPECT_FALSE(not_identity.is_identity());
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
    Mat<float, 2, 3> m({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
    EXPECT_FLOAT_EQ(m(0, 2), 3.0F);
    EXPECT_FLOAT_EQ(m(1, 2), 6.0F);
}

TEST(MatrixNonSquareTest, RectangularMultiplication)
{
    Mat<float, 2, 3> A({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
    Mat<float, 3, 2> B({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
    Mat<float, 2, 2> C = A * B;

    EXPECT_FLOAT_EQ(C(0, 0), 22.0F);
    EXPECT_FLOAT_EQ(C(0, 1), 28.0F);
}

// ===== Edge Cases =====

TEST(MatrixEdgeCaseTest, 1x1Matrix)
{
    Mat<float, 1, 1> m({42.0F});
    EXPECT_FLOAT_EQ(m(0, 0), 42.0F);
    EXPECT_FLOAT_EQ(m.determinant(), 42.0F);
}

TEST(MatrixEdgeCaseTest, LargeMatrix)
{
    Mat<float, 10, 10> m = Mat<float, 10, 10>::identity();
    EXPECT_FLOAT_EQ(m(5, 5), 1.0F);
    EXPECT_FLOAT_EQ(m(5, 6), 0.0F);
}
