// SPDX-License-Identifier: Apache-2.0
/// @file test_matrix_view.cpp
/// @brief Unit tests for MatrixView and ConstMatrixView
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <gtest/gtest.h>
#include <microla/matrix_view.hpp>

using namespace microla;

TEST(MatrixViewTest, ReadAndWriteParent)
{
    // clang-format off
    Mat<float, 4, 4> m({
        {1.0F,  2.0F,  3.0F,  4.0F},
        {5.0F,  6.0F,  7.0F,  8.0F},
        {9.0F, 10.0F, 11.0F, 12.0F},
        {13.0F, 14.0F, 15.0F, 16.0F}
    });
    // clang-format on

    // View the 2x2 block starting at (1,1): elements {6,7;10,11}
    MatrixView<float, 2, 2> v(m.data, 4, 1, 1);

    EXPECT_FLOAT_EQ(v(0, 0), m(1, 1));
    EXPECT_FLOAT_EQ(v(0, 1), m(1, 2));
    EXPECT_FLOAT_EQ(v(1, 0), m(2, 1));
    EXPECT_FLOAT_EQ(v(1, 1), m(2, 2));

    // Write through the view and ensure parent is updated
    v(0, 1) = 99.0F;
    EXPECT_FLOAT_EQ(m(1, 2), 99.0F);
}

TEST(MatrixViewTest, SetCopiesIntoParent)
{
    Mat<float, 4, 4> m = Mat<float, 4, 4>::zero();
    MatrixView<float, 2, 3> v(m.data, 4, 1, 0);

    // clang-format off
    Mat<float, 2, 3> src({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F}
    });
    // clang-format on
    v.set(src);

    // Parent should contain src values at rows 1..2 and cols 0..2
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            EXPECT_FLOAT_EQ(m(i + 1, j + 0), src(i, j));
        }
    }
}

TEST(MatrixViewTest, ToMatrixReturnsCopy)
{
    // clang-format off
    Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F}
    });
    // clang-format on
    MatrixView<float, 2, 2> v(m.data, 3, 0, 1);  // block {2,3;5,6}

    Mat<float, 2, 2> extracted = v.to_matrix();
    EXPECT_FLOAT_EQ(extracted(0, 0), 2.0F);
    EXPECT_FLOAT_EQ(extracted(0, 1), 3.0F);
    EXPECT_FLOAT_EQ(extracted(1, 0), 5.0F);
    EXPECT_FLOAT_EQ(extracted(1, 1), 6.0F);

    // Modify parent, ensure extracted copy is independent
    v(0, 0) = -1.0F;
    EXPECT_FLOAT_EQ(extracted(0, 0), 2.0F);
}

TEST(MatrixViewTest, FillWorks)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::ones();
    MatrixView<float, 2, 2> v(m.data, 3, 1, 1);

    v.fill(7.5F);

    EXPECT_FLOAT_EQ(m(1, 1), 7.5F);
    EXPECT_FLOAT_EQ(m(1, 2), 7.5F);
    EXPECT_FLOAT_EQ(m(2, 1), 7.5F);
    EXPECT_FLOAT_EQ(m(2, 2), 7.5F);
}

TEST(ConstMatrixViewTest, ToMatrixFromConstView)
{
    // clang-format off
    const Mat<float, 3, 3> m({
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F}
    });
    // clang-format on

    ConstMatrixView<float, 2, 2> cv(m.data, 3, 0, 0);
    Mat<float, 2, 2> out = cv.to_matrix();

    EXPECT_FLOAT_EQ(out(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(out(0, 1), 2.0F);
    EXPECT_FLOAT_EQ(out(1, 0), 4.0F);
    EXPECT_FLOAT_EQ(out(1, 1), 5.0F);
}
