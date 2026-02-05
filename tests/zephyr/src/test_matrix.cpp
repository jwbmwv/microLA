// SPDX-License-Identifier: MIT
/// @file test_matrix.cpp
/// @brief Zephyr ztest tests for Mat<T,M,N> class
/// @copyright Copyright (c) 2026 James Baldwin

#include <zephyr/ztest.h>
#include <microla/microla.hpp>
#include <cmath>

using namespace microla;

static const float epsilon = 1e-5f;

// Helper function for floating point comparison
static bool float_eq(float a, float b, float eps = epsilon)
{
    return std::fabs(a - b) < eps;
}

// ===== Construction Tests =====

ZTEST(microla_matrix, test_default_constructor)
{
    Mat<float, 2, 2> m;
    for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < 2; j++) {
            zassert_true(float_eq(m(i, j), 0.0f),
                "Default constructor should initialize to zero");
        }
    }
}

ZTEST(microla_matrix, test_identity_matrix)
{
    auto m = Mat<float, 3, 3>::identity();

    // Diagonal elements should be 1
    zassert_true(float_eq(m(0, 0), 1.0f), "Identity diagonal [0,0] failed");
    zassert_true(float_eq(m(1, 1), 1.0f), "Identity diagonal [1,1] failed");
    zassert_true(float_eq(m(2, 2), 1.0f), "Identity diagonal [2,2] failed");

    // Off-diagonal elements should be 0
    zassert_true(float_eq(m(0, 1), 0.0f), "Identity off-diagonal failed");
    zassert_true(float_eq(m(0, 2), 0.0f), "Identity off-diagonal failed");
    zassert_true(float_eq(m(1, 0), 0.0f), "Identity off-diagonal failed");
}

ZTEST(microla_matrix, test_array_constructor)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};

    zassert_true(float_eq(m(0, 0), 1.0f), "Array constructor [0,0] failed");
    zassert_true(float_eq(m(0, 1), 2.0f), "Array constructor [0,1] failed");
    zassert_true(float_eq(m(1, 0), 3.0f), "Array constructor [1,0] failed");
    zassert_true(float_eq(m(1, 1), 4.0f), "Array constructor [1,1] failed");
}

ZTEST(microla_matrix, test_copy_constructor)
{
    Mat<float, 2, 2> m1{1.0f, 2.0f, 3.0f, 4.0f};
    Mat<float, 2, 2> m2(m1);

    zassert_true(m1 == m2, "Copy constructor failed");
}

// ===== Accessor Tests =====

ZTEST(microla_matrix, test_element_access)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};

    zassert_true(float_eq(m(0, 0), 1.0f), "Element access [0,0] failed");
    zassert_true(float_eq(m(0, 1), 2.0f), "Element access [0,1] failed");
    zassert_true(float_eq(m(1, 0), 3.0f), "Element access [1,0] failed");
    zassert_true(float_eq(m(1, 1), 4.0f), "Element access [1,1] failed");
}

ZTEST(microla_matrix, test_row_col_methods)
{
    auto m = Mat<float, 3, 3>::identity();

    zassert_equal(m.rows(), 3, "rows() method failed");
    zassert_equal(m.cols(), 3, "cols() method failed");
}

// ===== Arithmetic Operations =====

ZTEST(microla_matrix, test_matrix_addition)
{
    Mat<float, 2, 2> m1{1.0f, 2.0f, 3.0f, 4.0f};
    Mat<float, 2, 2> m2{5.0f, 6.0f, 7.0f, 8.0f};

    Mat<float, 2, 2> result = m1 + m2;

    zassert_true(float_eq(result(0, 0), 6.0f), "Matrix addition [0,0] failed");
    zassert_true(float_eq(result(0, 1), 8.0f), "Matrix addition [0,1] failed");
    zassert_true(float_eq(result(1, 0), 10.0f), "Matrix addition [1,0] failed");
    zassert_true(float_eq(result(1, 1), 12.0f), "Matrix addition [1,1] failed");
}

ZTEST(microla_matrix, test_matrix_subtraction)
{
    Mat<float, 2, 2> m1{5.0f, 6.0f, 7.0f, 8.0f};
    Mat<float, 2, 2> m2{1.0f, 2.0f, 3.0f, 4.0f};

    Mat<float, 2, 2> result = m1 - m2;

    zassert_true(float_eq(result(0, 0), 4.0f), "Matrix subtraction [0,0] failed");
    zassert_true(float_eq(result(0, 1), 4.0f), "Matrix subtraction [0,1] failed");
    zassert_true(float_eq(result(1, 0), 4.0f), "Matrix subtraction [1,0] failed");
    zassert_true(float_eq(result(1, 1), 4.0f), "Matrix subtraction [1,1] failed");
}

ZTEST(microla_matrix, test_scalar_multiplication)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};

    Mat<float, 2, 2> result = m * 2.0f;

    zassert_true(float_eq(result(0, 0), 2.0f), "Scalar mult [0,0] failed");
    zassert_true(float_eq(result(0, 1), 4.0f), "Scalar mult [0,1] failed");
    zassert_true(float_eq(result(1, 0), 6.0f), "Scalar mult [1,0] failed");
    zassert_true(float_eq(result(1, 1), 8.0f), "Scalar mult [1,1] failed");
}

ZTEST(microla_matrix, test_matrix_multiplication)
{
    // Test 2x2 matrix multiplication
    Mat<float, 2, 2> m1{1.0f, 2.0f, 3.0f, 4.0f};
    Mat<float, 2, 2> m2{5.0f, 6.0f, 7.0f, 8.0f};

    Mat<float, 2, 2> result = m1 * m2;

    // [[1,2],[3,4]] * [[5,6],[7,8]] = [[19,22],[43,50]]
    zassert_true(float_eq(result(0, 0), 19.0f), "Matrix mult [0,0] failed");
    zassert_true(float_eq(result(0, 1), 22.0f), "Matrix mult [0,1] failed");
    zassert_true(float_eq(result(1, 0), 43.0f), "Matrix mult [1,0] failed");
    zassert_true(float_eq(result(1, 1), 50.0f), "Matrix mult [1,1] failed");
}

ZTEST(microla_matrix, test_matrix_vector_multiplication)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};
    Vec<float, 2> v(5.0f, 6.0f);

    Vec<float, 2> result = m * v;

    // [[1,2],[3,4]] * [5,6] = [17,39]
    zassert_true(float_eq(result[0], 17.0f), "Matrix-vector mult [0] failed");
    zassert_true(float_eq(result[1], 39.0f), "Matrix-vector mult [1] failed");
}

// ===== Matrix Operations =====

ZTEST(microla_matrix, test_transpose)
{
    Mat<float, 2, 3> m{1.0f, 2.0f, 3.0f,
                       4.0f, 5.0f, 6.0f};

    Mat<float, 3, 2> result = m.transpose();

    zassert_true(float_eq(result(0, 0), 1.0f), "Transpose [0,0] failed");
    zassert_true(float_eq(result(0, 1), 4.0f), "Transpose [0,1] failed");
    zassert_true(float_eq(result(1, 0), 2.0f), "Transpose [1,0] failed");
    zassert_true(float_eq(result(1, 1), 5.0f), "Transpose [1,1] failed");
    zassert_true(float_eq(result(2, 0), 3.0f), "Transpose [2,0] failed");
    zassert_true(float_eq(result(2, 1), 6.0f), "Transpose [2,1] failed");
}

ZTEST(microla_matrix, test_determinant_2x2)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};

    float det = m.determinant();

    // det = 1*4 - 2*3 = 4 - 6 = -2
    zassert_true(float_eq(det, -2.0f), "2x2 determinant failed");
}

ZTEST(microla_matrix, test_determinant_3x3)
{
    Mat<float, 3, 3> m{1.0f, 2.0f, 3.0f,
                       0.0f, 1.0f, 4.0f,
                       5.0f, 6.0f, 0.0f};

    float det = m.determinant();

    // det = 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5)
    //     = 1*(-24) - 2*(-20) + 3*(-5)
    //     = -24 + 40 - 15 = 1
    zassert_true(float_eq(det, 1.0f), "3x3 determinant failed");
}

ZTEST(microla_matrix, test_inverse_identity)
{
    auto m = Mat<float, 3, 3>::identity();
    auto inv = m.inverse();

    // Inverse of identity should be identity
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            if (i == j) {
                zassert_true(float_eq(inv(i, j), 1.0f), "Identity inverse diagonal failed");
            } else {
                zassert_true(float_eq(inv(i, j), 0.0f), "Identity inverse off-diagonal failed");
            }
        }
    }
}

ZTEST(microla_matrix, test_inverse_2x2)
{
    Mat<float, 2, 2> m{1.0f, 2.0f, 3.0f, 4.0f};

    Mat<float, 2, 2> inv = m.inverse();
    Mat<float, 2, 2> product = m * inv;

    // Product should be approximately identity
    zassert_true(float_eq(product(0, 0), 1.0f, 1e-4f), "Inverse [0,0] failed");
    zassert_true(float_eq(product(0, 1), 0.0f, 1e-4f), "Inverse [0,1] failed");
    zassert_true(float_eq(product(1, 0), 0.0f, 1e-4f), "Inverse [1,0] failed");
    zassert_true(float_eq(product(1, 1), 1.0f, 1e-4f), "Inverse [1,1] failed");
}

// ===== Comparison Operations =====

ZTEST(microla_matrix, test_equality)
{
    Mat<float, 2, 2> m1{1.0f, 2.0f, 3.0f, 4.0f};
    Mat<float, 2, 2> m2{1.0f, 2.0f, 3.0f, 4.0f};

    zassert_true(m1 == m2, "Matrix equality failed");
}

ZTEST(microla_matrix, test_inequality)
{
    Mat<float, 2, 2> m1{1.0f, 2.0f, 3.0f, 4.0f};
    Mat<float, 2, 2> m2{5.0f, 6.0f, 7.0f, 8.0f};

    zassert_true(m1 != m2, "Matrix inequality failed");
}

// Test suite setup
ZTEST_SUITE(microla_matrix, NULL, NULL, NULL, NULL, NULL);
