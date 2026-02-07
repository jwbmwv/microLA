// SPDX-License-Identifier: MIT
/// @file test_matrix_view.cpp
/// @brief Unit tests for MatrixView and ConstMatrixView
/// @copyright Copyright (c) 2026 James Baldwin

#include <gtest/gtest.h>
#include <microla/matrix_view.hpp>

using namespace microla;

TEST(MatrixViewTest, ReadAndWriteParent)
{
    Mat<float, 4, 4> m({
        1.0f, 2.0f, 3.0f, 4.0f,
        5.0f, 6.0f, 7.0f, 8.0f,
        9.0f, 10.0f, 11.0f, 12.0f,
        13.0f, 14.0f, 15.0f, 16.0f});

    // View the 2x2 block starting at (1,1): elements {6,7;10,11}
    MatrixView<float, 2, 2> v(m.data, 4, 1, 1);

    EXPECT_FLOAT_EQ(v(0, 0), m(1, 1));
    EXPECT_FLOAT_EQ(v(0, 1), m(1, 2));
    EXPECT_FLOAT_EQ(v(1, 0), m(2, 1));
    EXPECT_FLOAT_EQ(v(1, 1), m(2, 2));

    // Write through the view and ensure parent is updated
    v(0, 1) = 99.0f;
    EXPECT_FLOAT_EQ(m(1, 2), 99.0f);
}

TEST(MatrixViewTest, SetCopiesIntoParent)
{
    Mat<float, 4, 4> m = Mat<float, 4, 4>::zero();
    MatrixView<float, 2, 3> v(m.data, 4, 1, 0);

    Mat<float, 2, 3> src({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    v.set(src);

    // Parent should contain src values at rows 1..2 and cols 0..2
    for (std::uint32_t i = 0; i < 2; ++i)
    {
        for (std::uint32_t j = 0; j < 3; ++j)
        {
            EXPECT_FLOAT_EQ(m(i + 1, j + 0), src(i, j));
        }
    }
}

TEST(MatrixViewTest, ToMatrixReturnsCopy)
{
    Mat<float, 3, 3> m({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f});
    MatrixView<float, 2, 2> v(m.data, 3, 0, 1); // block {2,3;5,6}

    Mat<float, 2, 2> extracted = v.toMatrix();
    EXPECT_FLOAT_EQ(extracted(0, 0), 2.0f);
    EXPECT_FLOAT_EQ(extracted(0, 1), 3.0f);
    EXPECT_FLOAT_EQ(extracted(1, 0), 5.0f);
    EXPECT_FLOAT_EQ(extracted(1, 1), 6.0f);

    // Modify parent, ensure extracted copy is independent
    v(0, 0) = -1.0f;
    EXPECT_FLOAT_EQ(extracted(0, 0), 2.0f);
}

TEST(MatrixViewTest, FillWorks)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::ones();
    MatrixView<float, 2, 2> v(m.data, 3, 1, 1);

    v.fill(7.5f);

    EXPECT_FLOAT_EQ(m(1, 1), 7.5f);
    EXPECT_FLOAT_EQ(m(1, 2), 7.5f);
    EXPECT_FLOAT_EQ(m(2, 1), 7.5f);
    EXPECT_FLOAT_EQ(m(2, 2), 7.5f);
}

TEST(ConstMatrixViewTest, ToMatrixFromConstView)
{
    const Mat<float, 3, 3> m({
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f,
        7.0f, 8.0f, 9.0f});

    ConstMatrixView<float, 2, 2> cv(m.data, 3, 0, 0);
    Mat<float, 2, 2> out = cv.toMatrix();

    EXPECT_FLOAT_EQ(out(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(out(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(out(1, 0), 4.0f);
    EXPECT_FLOAT_EQ(out(1, 1), 5.0f);
}
