// SPDX-License-Identifier: Apache-2.0
/// @file test_vector_view.cpp
/// @brief Google tests for VectorView and ConstVectorView

#include <gtest/gtest.h>
#include <microla/microla.hpp>

using namespace microla;

TEST(VectorViewTest, ReadAndWriteParent)
{
    Vec<float, 4> parent;
    for (std::size_t i = 0; i < 4; ++i)
    {
        parent[i] = static_cast<float>(i);
    }

    VectorView<float, 2> view(parent.data, 4, 1);
    EXPECT_FLOAT_EQ(view[0], parent[1]);

    view[0] = 99.0F;
    EXPECT_FLOAT_EQ(parent[1], 99.0F);
}

TEST(VectorViewTest, SetCopiesIntoParent)
{
    // source vector
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

    EXPECT_FLOAT_EQ(parent[1], 1.0F);
    EXPECT_FLOAT_EQ(parent[2], 2.0F);
    EXPECT_FLOAT_EQ(parent[3], 3.0F);
}

TEST(VectorViewTest, ToVecReturnsCopy)
{
    Vec<float, 5> parent;
    for (std::size_t i = 0; i < 5; ++i)
    {
        parent[i] = static_cast<float>(i + 1);
    }

    ConstVectorView<float, 2> cview(parent.data, 5, 2);
    Vec<float, 2> copy = cview.to_vec();

    parent[2] = -1.0F;
    EXPECT_FALSE(copy[0] == parent[2]);
    EXPECT_FLOAT_EQ(copy[0], 3.0F);
}

TEST(VectorViewTest, FillWorks)
{
    Vec<float, 4> parent;
    for (std::size_t i = 0; i < 4; ++i)
    {
        parent[i] = 0.0F;
    }

    VectorView<float, 3> view(parent.data, 4, 0);
    view.fill(3.14F);

    EXPECT_FLOAT_EQ(parent[0], 3.14F);
    EXPECT_FLOAT_EQ(parent[1], 3.14F);
    EXPECT_FLOAT_EQ(parent[2], 3.14F);
}
