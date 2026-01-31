// SPDX-License-Identifier: Apache-2.0
// Additional unit tests: type traits and constexpr-friendly factories

#include <gtest/gtest.h>
#include <type_traits>
#include "microla/vector.hpp"
#include "microla/matrix.hpp"
#include "microla/quaternion.hpp"

using microla::Quaternion;
using microla::Vec;

TEST(Traits, VecIsTriviallyCopyableAndAligned)
{
    EXPECT_TRUE((std::is_trivially_copyable<Vec<float, 3>>::value));
    EXPECT_TRUE((std::is_standard_layout<Vec<float, 3>>::value));
    // Alignment expectation (should match alignas(16) used in Vec)
    EXPECT_EQ(alignof(Vec<float, 3>), static_cast<std::size_t>(16));
}

TEST(Traits, QuaternionTraits)
{
    EXPECT_TRUE((std::is_trivially_copyable<Quaternion<float>>::value));
    EXPECT_TRUE((std::is_standard_layout<Quaternion<float>>::value));
    EXPECT_EQ(alignof(Quaternion<float>), static_cast<std::size_t>(16));
}

TEST(ConstexprFactories, VecZeroConstexprUsage)
{
    constexpr auto v = Vec<int, 2>::zero();
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 0);
}

TEST(ConstexprFactories, VecFilledConstexprUsage)
{
    constexpr auto v = Vec<int, 3>::filled(7);
    EXPECT_EQ(v[0], 7);
    EXPECT_EQ(v[1], 7);
    EXPECT_EQ(v[2], 7);
}

TEST(ConstexprFactories, QuaternionIdentity)
{
    auto q = Quaternion<float>::identity();
    EXPECT_FLOAT_EQ(q.w(), 1.0F);
    EXPECT_FLOAT_EQ(q.x(), 0.0F);
    EXPECT_FLOAT_EQ(q.y(), 0.0F);
    EXPECT_FLOAT_EQ(q.z(), 0.0F);
}
