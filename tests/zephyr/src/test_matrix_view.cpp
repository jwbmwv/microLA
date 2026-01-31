// SPDX-License-Identifier: Apache-2.0
/// @file test_matrix_view.cpp
/// @brief Zephyr ztest tests for MatrixView and ConstMatrixView
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

static const float epsilon = 1e-6F;

static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

ZTEST(microla_matrix_view, test_read_and_write_parent)
{
    Mat<float, 4, 4> M;
    // fill parent with row-major incremental values
    for (std::size_t r = 0; r < 4; ++r)
    {
        for (std::size_t c = 0; c < 4; ++c)
        {
            M(r, c) = static_cast<float>(r * 10 + c);
        }
    }

    // create a 2x3 view starting at (1,1)
    MatrixView<float, 2, 3> view(M.data, M.cols(), 1, 1);

    // verify view reads correct parent values
    for (std::size_t r = 0; r < view.rows(); ++r)
    {
        for (std::size_t c = 0; c < view.cols(); ++c)
        {
            float expected = M(r + 1, c + 1);
            zassert_true(float_eq(view(r, c), expected), "View read mismatch");
        }
    }

    // write through view and verify parent changed
    view(0, 0) = 99.5F;
    zassert_true(float_eq(M(1, 1), 99.5F), "Write through view failed");
}

ZTEST(microla_matrix_view, test_set_copies_into_parent)
{
    // clang-format off
    Mat<float, 3, 2> source{
        {1.0F, 2.0F},
        {3.0F, 4.0F},
        {5.0F, 6.0F}};
    // clang-format on

    Mat<float, 4, 4> parent;  // zero-initialized
    auto view = MatrixView<float, 3, 2>(parent.data, parent.cols(), 0, 1);

    view.set(source);

    // verify parent region contains source values
    for (std::size_t r = 0; r < 3; ++r)
    {
        for (std::size_t c = 0; c < 2; ++c)
        {
            zassert_true(float_eq(parent(r, c + 1), source(r, c)), "set() did not copy correctly");
        }
    }
}

ZTEST(microla_matrix_view, test_toMatrix_returns_copy)
{
    Mat<float, 4, 4> parent;
    for (std::size_t r = 0; r < 4; ++r)
    {
        for (std::size_t c = 0; c < 4; ++c)
        {
            parent(r, c) = static_cast<float>(r * 4 + c + 1);
        }
    }

    auto view = ConstMatrixView<float, 2, 2>(parent.data, parent.cols(), 2, 2);
    Mat<float, 2, 2> copy = view.to_matrix();

    // mutate parent and ensure copy does not change
    parent(2, 2) = -1.0F;
    zassert_true(!float_eq(copy(0, 0), parent(2, 2)), "toMatrix did not return an independent copy");

    // verify copy contents equal original region values
    zassert_true(float_eq(copy(0, 0), 11.0F), "toMatrix content mismatch");
}

ZTEST(microla_matrix_view, test_fill_works)
{
    Mat<float, 4, 4> parent;
    auto view = MatrixView<float, 2, 2>(parent.data, parent.cols(), 1, 2);

    view.fill(3.14F);

    for (std::size_t r = 0; r < 2; ++r)
    {
        for (std::size_t c = 0; c < 2; ++c)
        {
            zassert_true(float_eq(parent(r + 1, c + 2), 3.14F), "fill() did not populate parent region");
        }
    }
}

ZTEST(microla_matrix_view, test_const_view_toMatrix)
{
    // clang-format off
    Mat<float, 3, 3> parent{
        {1.0F, 2.0F, 3.0F},
        {4.0F, 5.0F, 6.0F},
        {7.0F, 8.0F, 9.0F}};
    // clang-format on
    const Mat<float, 3, 3>& cref = parent;

    ConstMatrixView<float, 2, 2> cview(cref.data, cref.cols(), 0, 0);
    Mat<float, 2, 2> m = cview.to_matrix();

    zassert_true(float_eq(m(0, 0), 1.0F), "Const toMatrix value mismatch");
    zassert_true(float_eq(m(1, 1), 5.0F), "Const toMatrix value mismatch");
}

ZTEST_SUITE(microla_matrix_view, NULL, NULL, NULL, NULL, NULL);
