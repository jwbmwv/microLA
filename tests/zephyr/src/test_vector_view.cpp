// SPDX-License-Identifier: Apache-2.0
/// @file test_vector_view.cpp
/// @brief Zephyr ztest tests for VectorView and ConstVectorView

#include <zephyr/ztest.h>
#include <microla/microla.hpp>

using namespace microla;

static const float epsilon_v = 1e-6F;

static bool float_eq_v(float a, float b, float eps = epsilon_v)
{
    return std::fabs(a - b) < eps;
}

ZTEST(microla_vector_view, test_read_and_write_parent)
{
    Vec<float, 4> parent;
    for (std::size_t i = 0; i < 4; ++i)
    {
        parent[i] = static_cast<float>(i);
    }

    VectorView<float, 2> view(parent.data, 4, 1);
    zassert_true(float_eq_v(view[0], parent[1]), "View read mismatch");

    view[0] = 11.5F;
    zassert_true(float_eq_v(parent[1], 11.5F), "Write through view failed");
}

ZTEST(microla_vector_view, test_set_copies_into_parent)
{
    Vec<float, 3> source;
    source[0] = 1.0F;
    source[1] = 2.0F;
    source[2] = 3.0F;

    Vec<float, 5> parent;
    for (std::size_t i = 0; i < 5; ++i)
    {
        parent[i] = 0.0F;
    }

    VectorView<float, 3> view(parent.data, 5, 1);
    view.set(source);

    for (std::size_t i = 0; i < 3; ++i)
    {
        zassert_true(float_eq_v(parent[1 + i], source[i]), "set() did not copy correctly");
    }
}

ZTEST(microla_vector_view, test_toVec_returns_copy)
{
    Vec<float, 5> parent;
    for (std::size_t i = 0; i < 5; ++i)
    {
        parent[i] = static_cast<float>(i + 1);
    }

    ConstVectorView<float, 2> cview(parent.data, 5, 2);
    Vec<float, 2> copy = cview.to_vec();

    parent[2] = -1.0F;
    zassert_true(!float_eq_v(copy[0], parent[2]), "toVec did not return an independent copy");
    zassert_true(float_eq_v(copy[0], 3.0F), "toVec content mismatch");
}

ZTEST_SUITE(microla_vector_view, NULL, NULL, NULL, NULL, NULL);
