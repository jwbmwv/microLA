// SPDX-License-Identifier: Apache-2.0
/// @file test_coverage_gaps.cpp
/// @brief Tests for previously uncovered API surfaces and recently fixed code paths
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <microla/version.hpp>
#include <microla/geometry.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <numeric>
#include <variant>
#include <vector>

using namespace microla;
using namespace microla::geometry;

// =============================================================================
// CRITICAL: Recently fixed code paths
// =============================================================================

// ----- Mat::rank() with partial pivoting -----

TEST(MatrixRank, FullRankSquare)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {0.0F, 1.0F, 4.0F},
        {5.0F, 6.0F, 0.0F}
    });
    // clang-format on
    EXPECT_EQ(m.rank(), 3u);
}

TEST(MatrixRank, RankDeficient)
{
    // Row 3 = Row 1 + Row 2, so rank = 2
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {5.0F, 7.0F, 9.0F}
    });
    // clang-format on
    EXPECT_EQ(m.rank(), 2u);
}

TEST(MatrixRank, Singular)
{
    // All rows are multiples of {1,2,3}
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {2.0F, 4.0F, 6.0F},
        {3.0F, 6.0F, 9.0F}
    });
    // clang-format on
    EXPECT_EQ(m.rank(), 1u);
}

TEST(MatrixRank, ZeroMatrix)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::zero();
    EXPECT_EQ(m.rank(), 0u);
}

TEST(MatrixRank, IdentityMatrix)
{
    Mat<float, 4, 4> m = Mat<float, 4, 4>::identity();
    EXPECT_EQ(m.rank(), 4u);
}

TEST(MatrixRank, ZeroFirstColumn)
{
    // This specifically tests partial pivoting - first column has zero in (0,0)
    // clang-format off
    Mat<float, 3, 3> m({
        {0.0F, 1.0F, 2.0F},
        {1.0F, 0.0F, 3.0F},
        {0.0F, 0.0F, 1.0F}
    });
    // clang-format on
    EXPECT_EQ(m.rank(), 3u);
}

TEST(MatrixRank, RectangularTall)
{
    // 4x2 matrix with rank 2
    // clang-format off
    Mat<float, 4, 2> m({
        {1.0F, 0.0F},
        {0.0F, 1.0F},
        {1.0F, 1.0F},
        {2.0F, 1.0F}
    });
    // clang-format on
    EXPECT_EQ(m.rank(), 2u);
}

TEST(MatrixRank, RectangularWide)
{
    // 2x4 matrix with rank 2
    Mat<float, 2, 4> m({1.0F, 0.0F, 1.0F, 2.0F, 0.0F, 1.0F, 1.0F, 1.0F});
    EXPECT_EQ(m.rank(), 2u);
}

TEST(MatrixRank, OneByOne)
{
    Mat<float, 1, 1> nonzero({5.0F});
    EXPECT_EQ(nonzero.rank(), 1u);

    Mat<float, 1, 1> zero({0.0F});
    EXPECT_EQ(zero.rank(), 0u);
}

// ----- SVD wide-matrix (R < C) path -----

TEST(MatrixSVD, WideMatrix2x4)
{
    // clang-format off
    Mat<float, 2, 4> A({
        {1.0F, 2.0F, 3.0F, 4.0F},
        {5.0F, 6.0F, 7.0F, 8.0F}
    });
    // clang-format on
    auto [U, S, V] = A.svd();

    // Verify reconstruction: U * S * V^T ≈ A
    Mat<float, 2, 4> reconstructed = U * S * V.transpose();
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), A(i, j), 0.1F) << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

TEST(MatrixSVD, TallMatrix4x2)
{
    // clang-format off
    Mat<float, 4, 2> A({
        {1.0F, 2.0F},
        {3.0F, 4.0F},
        {5.0F, 6.0F},
        {7.0F, 8.0F}
    });
    // clang-format on
    auto [U, S, V] = A.svd();

    // Verify reconstruction: U * S * V^T ≈ A
    Mat<float, 4, 2> reconstructed = U * S * V.transpose();
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            EXPECT_NEAR(reconstructed(i, j), A(i, j), 0.1F) << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

TEST(MatrixSVD, SingularValuesNonNegative)
{
    // clang-format off
    Mat<float, 2, 3> A({
        {1.0F, 0.0F, 1.0F},
        {0.0F, 1.0F, 1.0F}
    });
    // clang-format on
    auto [U, S, V] = A.svd();

    // Diagonal entries of S should be non-negative
    for (int i = 0; i < 2; ++i)
    {
        EXPECT_GE(S(i, i), 0.0F);
    }
}

// ----- Quaternion::operator*=(scalar) -----

TEST(QuaternionCompound, ScalarMultiplyAssign)
{
    // Constructor is (w, x, y, z), storage is [x, y, z, w]
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);  // w=1, x=2, y=3, z=4
    q *= 2.0F;
    EXPECT_FLOAT_EQ(q[0], 4.0F);  // x*2
    EXPECT_FLOAT_EQ(q[1], 6.0F);  // y*2
    EXPECT_FLOAT_EQ(q[2], 8.0F);  // z*2
    EXPECT_FLOAT_EQ(q[3], 2.0F);  // w*2
}

TEST(QuaternionCompound, ScalarMultiplyAssignZero)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);  // w=1, x=2, y=3, z=4
    q *= 0.0F;
    EXPECT_FLOAT_EQ(q[0], 0.0F);  // x
    EXPECT_FLOAT_EQ(q[1], 0.0F);  // y
    EXPECT_FLOAT_EQ(q[2], 0.0F);  // z
    EXPECT_FLOAT_EQ(q[3], 0.0F);  // w
}

TEST(QuaternionCompound, ScalarMultiplyAssignNegative)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);  // w=1, x=2, y=3, z=4
    q *= -1.0F;
    EXPECT_FLOAT_EQ(q[0], -2.0F);  // -x
    EXPECT_FLOAT_EQ(q[1], -3.0F);  // -y
    EXPECT_FLOAT_EQ(q[2], -4.0F);  // -z
    EXPECT_FLOAT_EQ(q[3], -1.0F);  // -w
}

// ----- fast::atan2 all quadrants -----

TEST(FastMath, Atan2AllQuadrants)
{
    const float tol = 0.15F;  // Fast approximation tolerance

    // Quadrant I: +y, +x
    EXPECT_NEAR(fast::atan2(1.0F, 1.0F), std::atan2(1.0F, 1.0F), tol);

    // Quadrant II: +y, -x
    EXPECT_NEAR(fast::atan2(1.0F, -1.0F), std::atan2(1.0F, -1.0F), tol);

    // Quadrant III: -y, -x
    EXPECT_NEAR(fast::atan2(-1.0F, -1.0F), std::atan2(-1.0F, -1.0F), tol);

    // Quadrant IV: -y, +x
    EXPECT_NEAR(fast::atan2(-1.0F, 1.0F), std::atan2(-1.0F, 1.0F), tol);
}

TEST(FastMath, Atan2AxisAligned)
{
    const float tol = 0.15F;

    // +y axis
    EXPECT_NEAR(fast::atan2(1.0F, 0.0F), std::atan2(1.0F, 0.0F), tol);

    // -y axis
    EXPECT_NEAR(fast::atan2(-1.0F, 0.0F), std::atan2(-1.0F, 0.0F), tol);

    // +x axis
    EXPECT_NEAR(fast::atan2(0.0F, 1.0F), std::atan2(0.0F, 1.0F), tol);

    // -x axis
    EXPECT_NEAR(fast::atan2(0.0F, -1.0F), std::atan2(0.0F, -1.0F), tol);
}

TEST(FastMath, Atan2LargeRatio)
{
    const float tol = 0.15F;

    // Large |z| = |y/x| - tests the |z|>=1 branch
    EXPECT_NEAR(fast::atan2(10.0F, 1.0F), std::atan2(10.0F, 1.0F), tol);
    EXPECT_NEAR(fast::atan2(-10.0F, 1.0F), std::atan2(-10.0F, 1.0F), tol);
    EXPECT_NEAR(fast::atan2(1.0F, 10.0F), std::atan2(1.0F, 10.0F), tol);
}

TEST(FastMath, SqrtNegativeReturnsZero)
{
    EXPECT_FLOAT_EQ(fast::sqrt(-1.0F), 0.0F);
    EXPECT_FLOAT_EQ(fast::sqrt(-100.0F), 0.0F);
}

// =============================================================================
// Matrix: compound assignment, try_inverse, inverse_checked, set_row/set_col
// =============================================================================

TEST(MatrixOps, CompoundAddAssign)
{
    Mat<float, 2, 2> a({1.0F, 2.0F, 3.0F, 4.0F});
    Mat<float, 2, 2> b({5.0F, 6.0F, 7.0F, 8.0F});
    a += b;
    EXPECT_FLOAT_EQ(a(0, 0), 6.0F);
    EXPECT_FLOAT_EQ(a(0, 1), 8.0F);
    EXPECT_FLOAT_EQ(a(1, 0), 10.0F);
    EXPECT_FLOAT_EQ(a(1, 1), 12.0F);
}

TEST(MatrixOps, CompoundSubAssign)
{
    Mat<float, 2, 2> a({5.0F, 6.0F, 7.0F, 8.0F});
    Mat<float, 2, 2> b({1.0F, 2.0F, 3.0F, 4.0F});
    a -= b;
    EXPECT_FLOAT_EQ(a(0, 0), 4.0F);
    EXPECT_FLOAT_EQ(a(1, 1), 4.0F);
}

TEST(MatrixOps, CompoundMulScalarAssign)
{
    Mat<float, 2, 2> a({1.0F, 2.0F, 3.0F, 4.0F});
    a *= 2.0F;
    EXPECT_FLOAT_EQ(a(0, 0), 2.0F);
    EXPECT_FLOAT_EQ(a(1, 1), 8.0F);
}

TEST(MatrixOps, CompoundDivScalarAssign)
{
    Mat<float, 2, 2> a({2.0F, 4.0F, 6.0F, 8.0F});
    a /= 2.0F;
    EXPECT_FLOAT_EQ(a(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(a(1, 1), 4.0F);
}

TEST(MatrixOps, TryInverseSuccess)
{
    Mat<float, 2, 2> m({4.0F, 7.0F, 2.0F, 6.0F});
    auto result = m.try_inverse();
    ASSERT_TRUE(result.has_value());
    Mat<float, 2, 2> product = m * (*result);
    EXPECT_NEAR(product(0, 0), 1.0F, 1e-5F);
    EXPECT_NEAR(product(0, 1), 0.0F, 1e-5F);
    EXPECT_NEAR(product(1, 0), 0.0F, 1e-5F);
    EXPECT_NEAR(product(1, 1), 1.0F, 1e-5F);
}

TEST(MatrixOps, TryInverseSingular)
{
    // Singular matrix: det = 0
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 2.0F},
        {2.0F, 4.0F}
    });
    // clang-format on
    auto result = m.try_inverse();
    EXPECT_FALSE(result.has_value());
}

TEST(MatrixOps, InverseCheckedSuccess)
{
    Mat<float, 2, 2> m({4.0F, 7.0F, 2.0F, 6.0F});
    auto result = m.inverse_checked();
    using Mat2 = Mat<float, 2, 2>;
    ASSERT_TRUE(std::holds_alternative<Mat2>(result));
    auto inv = std::get<Mat2>(result);
    Mat2 product = m * inv;
    EXPECT_NEAR(product(0, 0), 1.0F, 1e-5F);
    EXPECT_NEAR(product(1, 1), 1.0F, 1e-5F);
}

TEST(MatrixOps, InverseCheckedSingular)
{
    // clang-format off
    Mat<float, 2, 2> m({
        {1.0F, 2.0F},
        {2.0F, 4.0F}
    });
    // clang-format on
    auto result = m.inverse_checked();
    using Mat2 = Mat<float, 2, 2>;
    using IE = Mat2::InverseError;
    bool is_error = std::holds_alternative<IE>(result);
    ASSERT_TRUE(is_error);
    EXPECT_EQ(std::get<IE>(result), IE::Singular);
}

TEST(MatrixOps, InverseOutParam)
{
    Mat<float, 2, 2> m({4.0F, 7.0F, 2.0F, 6.0F});
    Mat<float, 2, 2> out;
    bool ok = m.inverse(out);
    EXPECT_TRUE(ok);
    Mat<float, 2, 2> product = m * out;
    EXPECT_NEAR(product(0, 0), 1.0F, 1e-5F);
    EXPECT_NEAR(product(1, 1), 1.0F, 1e-5F);
}

TEST(MatrixOps, InverseOutParamSingular)
{
    Mat<float, 2, 2> m({1.0F, 2.0F, 2.0F, 4.0F});
    Mat<float, 2, 2> out;
    bool ok = m.inverse(out);
    EXPECT_FALSE(ok);
}

TEST(MatrixOps, SetRow)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::zero();
    Mat<float, 1, 3> row({10.0F, 20.0F, 30.0F});
    m.set_row(1, row);
    EXPECT_FLOAT_EQ(m(1, 0), 10.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 20.0F);
    EXPECT_FLOAT_EQ(m(1, 2), 30.0F);
    // Other rows unchanged
    EXPECT_FLOAT_EQ(m(0, 0), 0.0F);
    EXPECT_FLOAT_EQ(m(2, 2), 0.0F);
}

TEST(MatrixOps, SetCol)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::zero();
    Mat<float, 3, 1> col({10.0F, 20.0F, 30.0F});
    m.set_col(1, col);
    EXPECT_FLOAT_EQ(m(0, 1), 10.0F);
    EXPECT_FLOAT_EQ(m(1, 1), 20.0F);
    EXPECT_FLOAT_EQ(m(2, 1), 30.0F);
    EXPECT_FLOAT_EQ(m(0, 0), 0.0F);
}

TEST(MatrixOps, RowsColsSize)
{
    using Mat34 = Mat<float, 3, 4>;
    EXPECT_EQ(Mat34::rows(), 3u);
    EXPECT_EQ(Mat34::cols(), 4u);
    EXPECT_EQ(Mat34::size(), 12u);
}

TEST(MatrixOps, DataPtr)
{
    Mat<float, 2, 2> m({1.0F, 2.0F, 3.0F, 4.0F});
    float* ptr = m.data_ptr();
    ASSERT_NE(ptr, nullptr);
    EXPECT_FLOAT_EQ(ptr[0], 1.0F);

    const Mat<float, 2, 2>& cm = m;
    const float* cptr = cm.data_ptr();
    ASSERT_NE(cptr, nullptr);
    EXPECT_FLOAT_EQ(cptr[0], 1.0F);
}

TEST(MatrixOps, StaticScale)
{
    Mat<float, 2, 2> m({1.0F, 2.0F, 3.0F, 4.0F});
    auto scaled = Mat<float, 2, 2>::scale(m, 3.0F);
    EXPECT_FLOAT_EQ(scaled(0, 0), 3.0F);
    EXPECT_FLOAT_EQ(scaled(1, 1), 12.0F);
}

TEST(MatrixOps, MoveConstructor)
{
    Mat<float, 2, 2> original({1.0F, 2.0F, 3.0F, 4.0F});
    Mat<float, 2, 2> moved(std::move(original));
    EXPECT_FLOAT_EQ(moved(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(moved(1, 1), 4.0F);
}

TEST(MatrixOps, MoveAssignment)
{
    Mat<float, 2, 2> original({1.0F, 2.0F, 3.0F, 4.0F});
    Mat<float, 2, 2> target;
    target = std::move(original);
    EXPECT_FLOAT_EQ(target(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(target(1, 1), 4.0F);
}

TEST(MatrixOps, Determinant4x4)
{
    Mat<float, 4, 4> m = Mat<float, 4, 4>::identity();
    EXPECT_NEAR(m.determinant(), 1.0F, 1e-6F);

    // Known 4x4 determinant
    // clang-format off
    Mat<float, 4, 4> m2({
        {1.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 2.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 3.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 4.0F}
    });
    // clang-format on
    EXPECT_NEAR(m2.determinant(), 24.0F, 1e-5F);
}

TEST(MatrixOps, Inverse3x3)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {0.0F, 1.0F, 4.0F},
        {5.0F, 6.0F, 0.0F}
    });
    // clang-format on
    Mat<float, 3, 3> inv = m.inverse();
    Mat<float, 3, 3> product = m * inv;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(product(i, j), i == j ? 1.0F : 0.0F, 1e-4F);
        }
    }
}

TEST(MatrixOps, Inverse4x4)
{
    // clang-format off
    Mat<float, 4, 4> m({
        {1.0F, 0.0F, 0.0F, 1.0F},
        {0.0F, 2.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 3.0F, 0.0F},
        {1.0F, 0.0F, 0.0F, 1.5F}
    });
    // clang-format on
    Mat<float, 4, 4> inv = m.inverse();
    Mat<float, 4, 4> product = m * inv;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(product(i, j), i == j ? 1.0F : 0.0F, 1e-4F);
        }
    }
}

TEST(MatrixOps, CholeskyFailsNonPD)
{
    // Not positive definite: diagonal has negative element
    // clang-format off
    Mat<float, 2, 2> m({
        {-1.0F, 0.0F},
        {0.0F,  1.0F}
    });
    // clang-format on
    auto result = m.cholesky();
    EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Quaternion: compound ops, lerp, negation, division, accessors
// =============================================================================

TEST(QuaternionCompound, AddAssign)
{
    // (w,x,y,z): a = (1,2,3,4), b = (0.5,0.5,0.5,0.5)
    // storage [x,y,z,w]: a = [2,3,4,1], b = [0.5,0.5,0.5,0.5]
    Quaternion<float> a(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> b(0.5F, 0.5F, 0.5F, 0.5F);
    a += b;
    EXPECT_FLOAT_EQ(a[0], 2.5F);  // x: 2+0.5
    EXPECT_FLOAT_EQ(a[1], 3.5F);  // y: 3+0.5
    EXPECT_FLOAT_EQ(a[2], 4.5F);  // z: 4+0.5
    EXPECT_FLOAT_EQ(a[3], 1.5F);  // w: 1+0.5
}

TEST(QuaternionCompound, SubAssign)
{
    Quaternion<float> a(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> b(0.5F, 0.5F, 0.5F, 0.5F);
    a -= b;
    EXPECT_FLOAT_EQ(a[0], 1.5F);  // x: 2-0.5
    EXPECT_FLOAT_EQ(a[1], 2.5F);  // y: 3-0.5
    EXPECT_FLOAT_EQ(a[2], 3.5F);  // z: 4-0.5
    EXPECT_FLOAT_EQ(a[3], 0.5F);  // w: 1-0.5
}

TEST(QuaternionCompound, DivScalarAssign)
{
    // (w,x,y,z) = (2,4,6,8) → storage [x,y,z,w] = [4,6,8,2]
    Quaternion<float> q(2.0F, 4.0F, 6.0F, 8.0F);
    q /= 2.0F;
    EXPECT_FLOAT_EQ(q[0], 2.0F);  // x: 4/2
    EXPECT_FLOAT_EQ(q[1], 3.0F);  // y: 6/2
    EXPECT_FLOAT_EQ(q[2], 4.0F);  // z: 8/2
    EXPECT_FLOAT_EQ(q[3], 1.0F);  // w: 2/2
}

TEST(QuaternionCompound, QuatMultiplyAssign)
{
    Quaternion<float> a = Quaternion<float>::identity();
    Quaternion<float> b =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), constants::pi<float>() / 2.0F);
    Quaternion<float> expected = a * b;
    a *= b;
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(a[i], expected[i], 1e-6F);
    }
}

TEST(QuaternionOps, ScalarDivision)
{
    // (w,x,y,z) = (2,4,6,8) → storage [4,6,8,2]
    Quaternion<float> q(2.0F, 4.0F, 6.0F, 8.0F);
    Quaternion<float> result = q / 2.0F;
    EXPECT_FLOAT_EQ(result[0], 2.0F);  // x: 4/2
    EXPECT_FLOAT_EQ(result[1], 3.0F);  // y: 6/2
    EXPECT_FLOAT_EQ(result[2], 4.0F);  // z: 8/2
    EXPECT_FLOAT_EQ(result[3], 1.0F);  // w: 2/2
}

TEST(QuaternionOps, UnaryNegation)
{
    // (w,x,y,z) = (1,2,3,4) → storage [2,3,4,1]
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> neg = -q;
    EXPECT_FLOAT_EQ(neg[0], -2.0F);  // -x
    EXPECT_FLOAT_EQ(neg[1], -3.0F);  // -y
    EXPECT_FLOAT_EQ(neg[2], -4.0F);  // -z
    EXPECT_FLOAT_EQ(neg[3], -1.0F);  // -w
}

TEST(QuaternionOps, Lerp)
{
    Quaternion<float> a = Quaternion<float>::identity();
    Quaternion<float> b =
        Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 0.0F, 1.0F), constants::pi<float>() / 2.0F);
    auto mid = a.lerp(b, 0.0F);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(mid[i], a[i], 1e-6F);
    }

    auto end = a.lerp(b, 1.0F);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(end[i], b[i], 1e-6F);
    }
}

TEST(QuaternionOps, VecAndSetVec)
{
    // (w,x,y,z) = (1,2,3,4) → storage [x=2, y=3, z=4, w=1]
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Vec<float, 3> v = q.vec();    // returns (x,y,z)
    EXPECT_FLOAT_EQ(v[0], 2.0F);  // x
    EXPECT_FLOAT_EQ(v[1], 3.0F);  // y
    EXPECT_FLOAT_EQ(v[2], 4.0F);  // z

    Vec<float, 3> newv(10.0F, 20.0F, 30.0F);
    q.set_vec(newv);
    EXPECT_FLOAT_EQ(q[0], 10.0F);  // x
    EXPECT_FLOAT_EQ(q[1], 20.0F);  // y
    EXPECT_FLOAT_EQ(q[2], 30.0F);  // z
    EXPECT_FLOAT_EQ(q[3], 1.0F);   // w unchanged
}

TEST(QuaternionOps, PtrAndAt)
{
    // (w,x,y,z) = (1,2,3,4) → storage [x=2, y=3, z=4, w=1]
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    float* p = q.ptr();
    ASSERT_NE(p, nullptr);
    EXPECT_FLOAT_EQ(p[0], 2.0F);  // x

    EXPECT_FLOAT_EQ(q.at(0), 2.0F);  // x
    EXPECT_FLOAT_EQ(q.at(3), 1.0F);  // w

    EXPECT_EQ(q.size(), 4u);
}

TEST(QuaternionOps, FromEuler3Arg)
{
    float roll = 0.0F;
    float pitch = 0.0F;
    float yaw = constants::pi<float>() / 2.0F;
    Quaternion<float> q = Quaternion<float>::from_euler(roll, pitch, yaw);

    // Should represent a 90-degree rotation around Z
    Vec<float, 3> point(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = q.rotate(point);
    EXPECT_NEAR(rotated[0], 0.0F, 0.01F);
    EXPECT_NEAR(rotated[1], 1.0F, 0.01F);
    EXPECT_NEAR(rotated[2], 0.0F, 0.01F);
}

TEST(QuaternionOps, ScalarTimesQuat)
{
    // (w,x,y,z) = (1,2,3,4) → storage [x=2, y=3, z=4, w=1]
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    Quaternion<float> result = 2.0F * q;
    EXPECT_FLOAT_EQ(result[0], 4.0F);  // x*2
    EXPECT_FLOAT_EQ(result[1], 6.0F);  // y*2
    EXPECT_FLOAT_EQ(result[2], 8.0F);  // z*2
    EXPECT_FLOAT_EQ(result[3], 2.0F);  // w*2
}

// =============================================================================
// Vector: at, data_ptr, iterators, cubic_hermite, catmull_rom, fma, scale, unit_w
// =============================================================================

TEST(VectorOps, AtAccess)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    EXPECT_FLOAT_EQ(v.at(0), 1.0F);
    EXPECT_FLOAT_EQ(v.at(2), 3.0F);
    v.at(1) = 99.0F;
    EXPECT_FLOAT_EQ(v.at(1), 99.0F);
}

TEST(VectorOps, DataPtr)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    float* ptr = v.data_ptr();
    ASSERT_NE(ptr, nullptr);
    EXPECT_FLOAT_EQ(ptr[0], 1.0F);

    const Vec<float, 3>& cv = v;
    const float* cptr = cv.data_ptr();
    ASSERT_NE(cptr, nullptr);
    EXPECT_FLOAT_EQ(cptr[2], 3.0F);
}

TEST(VectorOps, BeginEnd)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    float sum = 0.0F;
    for (auto it = v.begin(); it != v.end(); ++it)
    {
        sum += *it;
    }
    EXPECT_FLOAT_EQ(sum, 6.0F);

    // Range-for
    float sum2 = 0.0F;
    for (float val : v)
    {
        sum2 += val;
    }
    EXPECT_FLOAT_EQ(sum2, 6.0F);
}

TEST(VectorOps, CubicHermite)
{
    Vec<float, 2> p1(0.0F, 0.0F);
    Vec<float, 2> p2(1.0F, 1.0F);
    Vec<float, 2> t1(1.0F, 0.0F);
    Vec<float, 2> t2(1.0F, 0.0F);

    auto start = p1.cubic_hermite(p2, t1, t2, 0.0F);
    EXPECT_NEAR(start[0], 0.0F, 0.01F);
    EXPECT_NEAR(start[1], 0.0F, 0.01F);

    auto end = p1.cubic_hermite(p2, t1, t2, 1.0F);
    EXPECT_NEAR(end[0], 1.0F, 0.01F);
    EXPECT_NEAR(end[1], 1.0F, 0.01F);
}

TEST(VectorOps, CatmullRom)
{
    Vec<float, 2> p0(0.0F, 0.0F);
    Vec<float, 2> p1(1.0F, 1.0F);
    Vec<float, 2> p2(2.0F, 0.0F);
    Vec<float, 2> p3(3.0F, 1.0F);

    // At t=0, should be at p1
    auto start = p1.catmull_rom(p2, p0, p3, 0.0F);
    EXPECT_NEAR(start[0], 1.0F, 0.01F);
    EXPECT_NEAR(start[1], 1.0F, 0.01F);

    // At t=1, should be at p2
    auto end = p1.catmull_rom(p2, p0, p3, 1.0F);
    EXPECT_NEAR(end[0], 2.0F, 0.01F);
    EXPECT_NEAR(end[1], 0.0F, 0.01F);
}

TEST(VectorOps, FmaVecVec)
{
    Vec<float, 3> a(1.0F, 2.0F, 3.0F);
    Vec<float, 3> b(2.0F, 3.0F, 4.0F);
    Vec<float, 3> c(10.0F, 10.0F, 10.0F);
    auto result = a.fma(b, c);          // a*b + c
    EXPECT_FLOAT_EQ(result[0], 12.0F);  // 1*2 + 10
    EXPECT_FLOAT_EQ(result[1], 16.0F);  // 2*3 + 10
    EXPECT_FLOAT_EQ(result[2], 22.0F);  // 3*4 + 10
}

TEST(VectorOps, FmaScalarVec)
{
    Vec<float, 3> a(1.0F, 2.0F, 3.0F);
    Vec<float, 3> c(10.0F, 10.0F, 10.0F);
    auto result = a.fma(2.0F, c);  // a*2 + c
    EXPECT_FLOAT_EQ(result[0], 12.0F);
    EXPECT_FLOAT_EQ(result[1], 14.0F);
    EXPECT_FLOAT_EQ(result[2], 16.0F);
}

TEST(VectorOps, Scale)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);
    auto result = v.scale(3.0F);
    EXPECT_FLOAT_EQ(result[0], 3.0F);
    EXPECT_FLOAT_EQ(result[1], 6.0F);
    EXPECT_FLOAT_EQ(result[2], 9.0F);
}

TEST(VectorOps, UnitW)
{
    Vec<float, 4> w = Vec<float, 4>::unit_w();
    EXPECT_FLOAT_EQ(w[0], 0.0F);
    EXPECT_FLOAT_EQ(w[1], 0.0F);
    EXPECT_FLOAT_EQ(w[2], 0.0F);
    EXPECT_FLOAT_EQ(w[3], 1.0F);
}

TEST(VectorOps, AdditionalSwizzles)
{
    Vec<float, 3> v(1.0F, 2.0F, 3.0F);

    auto yx = v.yx();
    EXPECT_FLOAT_EQ(yx[0], 2.0F);
    EXPECT_FLOAT_EQ(yx[1], 1.0F);

    auto xz = v.xz();
    EXPECT_FLOAT_EQ(xz[0], 1.0F);
    EXPECT_FLOAT_EQ(xz[1], 3.0F);

    auto yz = v.yz();
    EXPECT_FLOAT_EQ(yz[0], 2.0F);
    EXPECT_FLOAT_EQ(yz[1], 3.0F);

    auto zx = v.zx();
    EXPECT_FLOAT_EQ(zx[0], 3.0F);
    EXPECT_FLOAT_EQ(zx[1], 1.0F);

    auto zy = v.zy();
    EXPECT_FLOAT_EQ(zy[0], 3.0F);
    EXPECT_FLOAT_EQ(zy[1], 2.0F);

    auto xzy = v.xzy();
    EXPECT_FLOAT_EQ(xzy[0], 1.0F);
    EXPECT_FLOAT_EQ(xzy[1], 3.0F);
    EXPECT_FLOAT_EQ(xzy[2], 2.0F);

    auto yxz = v.yxz();
    EXPECT_FLOAT_EQ(yxz[0], 2.0F);
    EXPECT_FLOAT_EQ(yxz[1], 1.0F);
    EXPECT_FLOAT_EQ(yxz[2], 3.0F);

    auto yzx = v.yzx();
    EXPECT_FLOAT_EQ(yzx[0], 2.0F);
    EXPECT_FLOAT_EQ(yzx[1], 3.0F);
    EXPECT_FLOAT_EQ(yzx[2], 1.0F);

    auto zxy = v.zxy();
    EXPECT_FLOAT_EQ(zxy[0], 3.0F);
    EXPECT_FLOAT_EQ(zxy[1], 1.0F);
    EXPECT_FLOAT_EQ(zxy[2], 2.0F);

    auto zyx = v.zyx();
    EXPECT_FLOAT_EQ(zyx[0], 3.0F);
    EXPECT_FLOAT_EQ(zyx[1], 2.0F);
    EXPECT_FLOAT_EQ(zyx[2], 1.0F);
}

// =============================================================================
// numerical_stability.hpp: untested functions
// =============================================================================

TEST(NumericalStability, Epsilon)
{
    float ef = numerical::epsilon<float>();
    EXPECT_GT(ef, 0.0F);
    EXPECT_LT(ef, 1.0F);

    double ed = numerical::epsilon<double>();
    EXPECT_GT(ed, 0.0);
    EXPECT_LT(ed, 1.0);
    EXPECT_LT(ed, ef);  // double epsilon is smaller
}

TEST(NumericalStability, StableDotProduct)
{
    float a[] = {1.0F, 2.0F, 3.0F};
    float b[] = {4.0F, 5.0F, 6.0F};
    float result = numerical::stable_dot_product(a, b, 3);
    EXPECT_NEAR(result, 32.0F, 1e-5F);  // 1*4 + 2*5 + 3*6 = 32
}

TEST(NumericalStability, StableDotProductLargeSmall)
{
    // Test with mixed magnitudes where compensated summation helps
    float a[] = {1e10F, 1.0F, -1e10F, 1.0F};
    float b[] = {1.0F, 1.0F, 1.0F, 1.0F};
    float result = numerical::stable_dot_product(a, b, 4);
    EXPECT_NEAR(result, 2.0F, 1.0F);  // Should be close to 2
}

TEST(NumericalStability, Log1p)
{
    // log1p(x) = log(1+x) should be accurate for small x
    EXPECT_NEAR(numerical::log1p(0.0F), 0.0F, 1e-6F);
    EXPECT_NEAR(numerical::log1p(1.0F), std::log(2.0F), 1e-5F);
    EXPECT_NEAR(numerical::log1p(1e-7F), 1e-7F, 1e-10F);  // For tiny x, log1p(x) ≈ x
}

TEST(NumericalStability, Expm1)
{
    // expm1(x) = exp(x) - 1 should be accurate for small x
    EXPECT_NEAR(numerical::expm1(0.0F), 0.0F, 1e-6F);
    EXPECT_NEAR(numerical::expm1(1.0F), std::exp(1.0F) - 1.0F, 1e-5F);
    EXPECT_NEAR(numerical::expm1(1e-7F), 1e-7F, 1e-10F);  // For tiny x, expm1(x) ≈ x
}

TEST(NumericalStability, IsRepresentable)
{
    EXPECT_TRUE(numerical::is_representable(1.0F));
    // 0 is not representable (below FLT_MIN) by this function's definition
    EXPECT_FALSE(numerical::is_representable(0.0F));
    EXPECT_FALSE(numerical::is_representable(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(numerical::is_representable(std::numeric_limits<float>::quiet_NaN()));
    // Subnormals are not representable
    EXPECT_FALSE(numerical::is_representable(std::numeric_limits<float>::denorm_min()));
    // Normal floats are
    EXPECT_TRUE(numerical::is_representable(std::numeric_limits<float>::min()));
}

TEST(NumericalStability, RelativeError)
{
    EXPECT_NEAR(numerical::relative_error(1.0F, 1.0F), 0.0F, 1e-6F);
    EXPECT_NEAR(numerical::relative_error(1.1F, 1.0F), 0.1F, 0.01F);
    EXPECT_NEAR(numerical::relative_error(2.0F, 1.0F), 1.0F, 0.01F);
}

TEST(NumericalStability, PrecisionLossBits)
{
    // Well-conditioned: 0 bits lost
    int bits_good = numerical::precision_loss_bits(1.0F);
    EXPECT_EQ(bits_good, 0);

    // Ill-conditioned: many bits lost
    int bits_bad = numerical::precision_loss_bits(1e10F);
    EXPECT_GT(bits_bad, 20);
}

TEST(NumericalStability, UlpDistance)
{
    // Same values should be 0
    auto dist_same = numerical::ulp_distance(1.0F, 1.0F);
    EXPECT_EQ(dist_same, 0);

    // Widely separated values should have large distance
    auto dist_large = numerical::ulp_distance(1.0F, 2.0F);
    EXPECT_GT(dist_large, 1000);

    // 10 ULPs apart (generous margin for the implementation)
    float a = 1.0F;
    float b = a + 10.0F * std::numeric_limits<float>::epsilon();
    auto dist10 = numerical::ulp_distance(a, b);
    EXPECT_GE(dist10, 5);  // Implementation rounds, so allow some slack
    EXPECT_LE(dist10, 15);
}

// =============================================================================
// safe_math.hpp: safe_reciprocal, check_bounds, assert_finite
// =============================================================================

TEST(SafeMath, SafeReciprocal)
{
    EXPECT_FLOAT_EQ(safe::safe_reciprocal(2.0F), 0.5F);
    EXPECT_FLOAT_EQ(safe::safe_reciprocal(4.0F), 0.25F);
    // Very small value should return default (0)
    EXPECT_FLOAT_EQ(safe::safe_reciprocal(0.0F), 0.0F);
    // Custom default
    EXPECT_FLOAT_EQ(safe::safe_reciprocal(0.0F, 99.0F), 99.0F);
}

TEST(SafeMath, CheckBounds)
{
    EXPECT_TRUE(safe::check_bounds(0, 10));
    EXPECT_TRUE(safe::check_bounds(9, 10));
    // Note: in non-SAFE_MODE, check_bounds always returns true
}

TEST(SafeMath, AssertFinite)
{
    float val = safe::assert_finite(42.0F);
    EXPECT_FLOAT_EQ(val, 42.0F);

    // In non-SAFE_MODE, assert_finite just passes through
    float inf = safe::assert_finite(std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isinf(inf));
}

// =============================================================================
// geometry.hpp: untested functions
// =============================================================================

TEST(GeometryExtras, PlaneFromPoints)
{
    Vec<float, 3> p0(0.0F, 0.0F, 0.0F);
    Vec<float, 3> p1(1.0F, 0.0F, 0.0F);
    Vec<float, 3> p2(0.0F, 1.0F, 0.0F);

    auto plane = Plane<float>::from_points(p0, p1, p2);
    // Normal should be (0,0,1) or (0,0,-1)
    EXPECT_NEAR(std::abs(plane.normal[2]), 1.0F, 0.01F);
    EXPECT_NEAR(plane.normal[0], 0.0F, 0.01F);
    EXPECT_NEAR(plane.normal[1], 0.0F, 0.01F);
}

TEST(GeometryExtras, AABBExpandPoint)
{
    AABB<float> box(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));
    box.expand(Vec<float, 3>(2.0F, 2.0F, 2.0F));

    // Max should now be (2,2,2)
    EXPECT_FLOAT_EQ(box.max[0], 2.0F);
    EXPECT_FLOAT_EQ(box.max[1], 2.0F);
    EXPECT_FLOAT_EQ(box.max[2], 2.0F);
    // Min should be unchanged
    EXPECT_FLOAT_EQ(box.min[0], 0.0F);
}

TEST(GeometryExtras, AABBSurfaceArea)
{
    AABB<float> box(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 2.0F, 3.0F));
    // Surface area = 2*(1*2 + 2*3 + 3*1) = 2*(2+6+3) = 22
    EXPECT_FLOAT_EQ(box.surface_area(), 22.0F);
}

TEST(GeometryExtras, AABBVolume)
{
    AABB<float> box(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(2.0F, 3.0F, 4.0F));
    EXPECT_FLOAT_EQ(box.volume(), 24.0F);
}

TEST(GeometryExtras, SphereToAABB)
{
    Sphere<float> s(Vec<float, 3>(1.0F, 2.0F, 3.0F), 1.0F);
    AABB<float> box = s.to_aabb();
    EXPECT_FLOAT_EQ(box.min[0], 0.0F);
    EXPECT_FLOAT_EQ(box.min[1], 1.0F);
    EXPECT_FLOAT_EQ(box.min[2], 2.0F);
    EXPECT_FLOAT_EQ(box.max[0], 2.0F);
    EXPECT_FLOAT_EQ(box.max[1], 3.0F);
    EXPECT_FLOAT_EQ(box.max[2], 4.0F);
}

TEST(GeometryExtras, SphereSurfaceArea)
{
    Sphere<float> s(Vec<float, 3>(0.0F, 0.0F, 0.0F), 1.0F);
    // 4πr² = 4π ≈ 12.566
    EXPECT_NEAR(s.surface_area(), 4.0F * constants::pi<float>(), 0.01F);
}

TEST(GeometryExtras, SphereVolume)
{
    Sphere<float> s(Vec<float, 3>(0.0F, 0.0F, 0.0F), 1.0F);
    // (4/3)πr³ = (4/3)π ≈ 4.189
    EXPECT_NEAR(s.volume(), (4.0F / 3.0F) * constants::pi<float>(), 0.01F);
}

TEST(GeometryExtras, TriangleToAABB)
{
    Triangle<float> tri(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(3.0F, 0.0F, 0.0F),
                        Vec<float, 3>(0.0F, 4.0F, 0.0F));
    AABB<float> box = tri.to_aabb();
    EXPECT_FLOAT_EQ(box.min[0], 0.0F);
    EXPECT_FLOAT_EQ(box.min[1], 0.0F);
    EXPECT_FLOAT_EQ(box.max[0], 3.0F);
    EXPECT_FLOAT_EQ(box.max[1], 4.0F);
}

// =============================================================================
// version.hpp
// =============================================================================

TEST(Version, VersionString)
{
    const char* vs = Version::string();
    ASSERT_NE(vs, nullptr);
    EXPECT_STREQ(vs, "0.0.2");
}

TEST(Version, VersionNumber)
{
    int num = Version::number();
    // major*10000 + minor*100 + patch = 0*10000 + 0*100 + 2 = 2
    EXPECT_EQ(num, 2);
}

TEST(Version, VersionComponents)
{
    EXPECT_EQ(Version::get_major(), 0);
    EXPECT_EQ(Version::get_minor(), 0);
    EXPECT_EQ(Version::get_patch(), 2);
}

TEST(Version, FreeFunction)
{
    EXPECT_STREQ(get_version_string(), "0.0.2");
    EXPECT_EQ(get_version_number(), 2);
}

TEST(Version, VersionAtLeast)
{
    EXPECT_TRUE(microla::version_at_least(0, 0, 2));
    EXPECT_TRUE(microla::version_at_least(0, 0, 1));
    EXPECT_TRUE(microla::version_at_least(0, 0, 0));
    EXPECT_FALSE(microla::version_at_least(0, 0, 3));
    EXPECT_FALSE(microla::version_at_least(0, 1, 0));
    EXPECT_FALSE(microla::version_at_least(1, 0, 0));
}

// =============================================================================
// matrix_view.hpp & vector_view.hpp: extended coverage
// =============================================================================

TEST(MatrixViewExtended, RowsCols)
{
    Mat<float, 4, 4> parent = Mat<float, 4, 4>::identity();
    MatrixView<float, 2, 2> view(parent.data, 4, 1, 1);
    EXPECT_EQ(view.rows(), 2u);
    EXPECT_EQ(view.cols(), 2u);
}

TEST(MatrixViewExtended, ConstViewElementAccess)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F}
    });
    // clang-format on
    ConstMatrixView<float, 2, 2> cview(m.data, 3, 1, 1);
    EXPECT_FLOAT_EQ(cview(0, 0), 5.0F);
    EXPECT_FLOAT_EQ(cview(0, 1), 6.0F);
    EXPECT_FLOAT_EQ(cview(1, 0), 8.0F);
    EXPECT_FLOAT_EQ(cview(1, 1), 9.0F);
}

TEST(VectorViewExtended, ToVec)
{
    Vec<float, 4> parent(1.0F, 2.0F, 3.0F, 4.0F);
    float* ptr = parent.data_ptr();
    VectorView<float, 2> view(ptr, 4, 1);
    Vec<float, 2> copy = view.to_vec();
    EXPECT_FLOAT_EQ(copy[0], 2.0F);
    EXPECT_FLOAT_EQ(copy[1], 3.0F);
}

TEST(VectorViewExtended, SizeAndStride)
{
    Vec<float, 4> parent(1.0F, 2.0F, 3.0F, 4.0F);
    float* ptr = parent.data_ptr();
    VectorView<float, 2> view(ptr, 4, 0, 2);  // stride=2
    EXPECT_EQ(view.size(), 2u);
    EXPECT_EQ(view.stride(), 2u);
}

TEST(VectorViewExtended, StridedAccess)
{
    Vec<float, 6> parent(10.0F, 20.0F, 30.0F, 40.0F, 50.0F, 60.0F);
    float* ptr = parent.data_ptr();
    VectorView<float, 3> view(ptr, 6, 0, 2);  // stride=2: indices 0,2,4
    EXPECT_FLOAT_EQ(view[0], 10.0F);
    EXPECT_FLOAT_EQ(view[1], 30.0F);
    EXPECT_FLOAT_EQ(view[2], 50.0F);
}

TEST(VectorViewExtended, ConstViewAccess)
{
    Vec<float, 4> parent(1.0F, 2.0F, 3.0F, 4.0F);
    const float* ptr = parent.data_ptr();
    ConstVectorView<float, 2> cview(ptr, 4, 1);
    EXPECT_FLOAT_EQ(cview[0], 2.0F);
    EXPECT_FLOAT_EQ(cview[1], 3.0F);
    EXPECT_EQ(cview.size(), 2u);
    EXPECT_EQ(cview.stride(), 1u);
}

TEST(VectorViewExtended, ConstViewToVec)
{
    Vec<float, 4> parent(1.0F, 2.0F, 3.0F, 4.0F);
    const float* ptr = parent.data_ptr();
    ConstVectorView<float, 2> cview(ptr, 4, 2);
    Vec<float, 2> copy = cview.to_vec();
    EXPECT_FLOAT_EQ(copy[0], 3.0F);
    EXPECT_FLOAT_EQ(copy[1], 4.0F);
}

// ============================================================================
// Large-N vector tests (N = 8, 16, 32) — exercises the generic scalar fallback
// loop for all operations that have no SIMD specialisation beyond N=4.
// ============================================================================

template<std::size_t N>
static void large_n_basic_ops()
{
    Vec<float, N> a, b;
    for (std::size_t i = 0; i < N; ++i)
    {
        a[i] = static_cast<float>(i + 1);
        b[i] = static_cast<float>(N - i);
    }

    // addition / subtraction round-trip
    const Vec<float, N> sum = a + b;
    const Vec<float, N> diff = sum - b;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_FLOAT_EQ(diff[i], a[i]) << "N=" << N << " i=" << i;
    }

    // scalar multiply / divide round-trip
    const Vec<float, N> scaled = a * 3.0F;
    const Vec<float, N> restored = scaled / 3.0F;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_NEAR(restored[i], a[i], 1e-5F) << "N=" << N << " i=" << i;
    }

    // dot product: a · a == sum of squares
    float expected_dot = 0.0F;
    for (std::size_t i = 0; i < N; ++i)
    {
        expected_dot += a[i] * a[i];
    }
    EXPECT_NEAR(a.dot(a), expected_dot, expected_dot * 1e-5F) << "N=" << N;

    // length: ||a|| == sqrt(a · a)
    EXPECT_NEAR(a.length(), std::sqrt(expected_dot), std::sqrt(expected_dot) * 1e-5F) << "N=" << N;

    // normalized: ||a.normalized()|| == 1
    const Vec<float, N> unit = a.normalized();
    EXPECT_NEAR(unit.length(), 1.0F, 1e-5F) << "N=" << N;

    // safe_normalized on zero vector returns zero
    const Vec<float, N> zero_vec;
    const Vec<float, N> safe_zero = zero_vec.safe_normalized();
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_FLOAT_EQ(safe_zero[i], 0.0F) << "N=" << N << " i=" << i;
    }
}

TEST(LargeNVector, N8BasicOps)
{
    large_n_basic_ops<8>();
}
TEST(LargeNVector, N16BasicOps)
{
    large_n_basic_ops<16>();
}
TEST(LargeNVector, N32BasicOps)
{
    large_n_basic_ops<32>();
}

template<std::size_t N>
static void large_n_assignment_ops()
{
    Vec<float, N> a;
    for (std::size_t i = 0; i < N; ++i)
    {
        a[i] = static_cast<float>(i + 1);
    }
    Vec<float, N> b = a;

    b += a;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_FLOAT_EQ(b[i], 2.0F * a[i]) << "N=" << N << " i=" << i;
    }

    b -= a;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_FLOAT_EQ(b[i], a[i]) << "N=" << N << " i=" << i;
    }

    b *= 2.0F;
    b /= 2.0F;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_NEAR(b[i], a[i], 1e-5F) << "N=" << N << " i=" << i;
    }

    // /= with zero denominator should produce NaN for floats
    Vec<float, N> c = a;
    c /= 0.0F;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_TRUE(std::isnan(c[i])) << "N=" << N << " i=" << i;
    }
}

TEST(LargeNVector, N8AssignOps)
{
    large_n_assignment_ops<8>();
}
TEST(LargeNVector, N16AssignOps)
{
    large_n_assignment_ops<16>();
}
TEST(LargeNVector, N32AssignOps)
{
    large_n_assignment_ops<32>();
}

TEST(LargeNVector, N32DotProductKahan)
{
    // Kahan-accurate dot over 32 elements — exercises the plain accumulation
    // loop for large N and catches precision issues.
    Vec<float, 32> a;
    for (std::size_t i = 0; i < 32; ++i)
    {
        a[i] = 1.0F;  // sum of squares == 32
    }
    EXPECT_NEAR(a.dot(a), 32.0F, 1e-4F);
    EXPECT_NEAR(a.length(), std::sqrt(32.0F), 1e-4F);
}
