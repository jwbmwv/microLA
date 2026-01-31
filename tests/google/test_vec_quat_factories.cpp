// SPDX-License-Identifier: Apache-2.0
// Tests for Vec factory helpers and basic Quaternion identity behavior

#include <gtest/gtest.h>
#include "microla/vector.hpp"
#include "microla/matrix.hpp"
#include "microla/quaternion.hpp"

using microla::Quaternion;
using microla::Vec;

TEST(VectorFactories, Zero)
{
    auto v = Vec<float, 3>::zero();
    EXPECT_EQ(v[0], 0.0f);
    EXPECT_EQ(v[1], 0.0f);
    EXPECT_EQ(v[2], 0.0f);
}

TEST(VectorFactories, Filled)
{
    const float val = 3.5f;
    auto v = Vec<float, 4>::filled(val);
    for (std::size_t i = 0; i < Vec<float, 4>::size(); ++i)
    {
        EXPECT_EQ(v[i], val);
    }
}

TEST(VectorFactories, One)
{
    auto v = Vec<int, 2>::one();
    EXPECT_EQ(v[0], 1);
    EXPECT_EQ(v[1], 1);
}

TEST(VectorFactories, UnitAxes)
{
    auto ux = Vec<float, 3>::unit_x();
    EXPECT_EQ(ux[0], 1.0f);
    EXPECT_EQ(ux[1], 0.0f);
    EXPECT_EQ(ux[2], 0.0f);

    auto uy = Vec<float, 3>::unit_y();
    EXPECT_EQ(uy[0], 0.0f);
    EXPECT_EQ(uy[1], 1.0f);
    EXPECT_EQ(uy[2], 0.0f);

    auto uz = Vec<float, 3>::unit_z();
    EXPECT_EQ(uz[0], 0.0f);
    EXPECT_EQ(uz[1], 0.0f);
    EXPECT_EQ(uz[2], 1.0f);
}

TEST(QuaternionBasic, IdentityViaCtor)
{
    // Quaternion constructor maps (w, x, y, z) -> data{x, y, z, w}
    Quaternion<float> q(1.0f, 0.0f, 0.0f, 0.0f);
    // w is stored in data[3]
    EXPECT_EQ(q[3], 1.0f);
    EXPECT_EQ(q[0], 0.0f);
    EXPECT_EQ(q[1], 0.0f);
    EXPECT_EQ(q[2], 0.0f);
}
