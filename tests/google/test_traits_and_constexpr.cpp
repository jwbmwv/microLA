// SPDX-License-Identifier: Apache-2.0
// Additional unit tests: type traits and constexpr-friendly factories

#include <gtest/gtest.h>
#include <type_traits>
#include "microla/vector.hpp"
#include "microla/matrix.hpp"
#include "microla/quaternion.hpp"
#include "microla/microla.hpp"

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

TEST(ConstexprFactories, FloatArithmeticFactories)
{
    constexpr auto sum = Vec<float, 3>{1, 2, 3} + Vec<float, 3>{4, 5, 6};
    constexpr auto scaled = (sum - Vec<float, 3>{1, 1, 1}) * 2.0F / 2.0F;
    constexpr auto zero = microla::Mat<float, 2, 2>::zero();
    constexpr auto ones = microla::Mat<float, 2, 2>::ones();
    static_assert(sum[0] == 5 && scaled[2] == 8);
    static_assert(zero(0, 0) == 0 && ones(1, 1) == 1);
    EXPECT_FLOAT_EQ(scaled[0], 4.0F);
}

TEST(ConstexprFactories, BoundedAngleWrapping)
{
    constexpr float angle = microla::wrap_pi(1e30F);
    static_assert(angle >= -microla::constants::pi<float>() && angle <= microla::constants::pi<float>());
    EXPECT_TRUE(std::isfinite(microla::wrap_pi(1e30F)));
    EXPECT_TRUE(std::isnan(microla::wrap_pi(std::numeric_limits<float>::infinity())));
    EXPECT_TRUE(std::isnan(microla::wrap_two_pi(std::numeric_limits<float>::quiet_NaN())));
    EXPECT_GE(microla::wrap_two_pi(-1e30F), 0.0F);
    EXPECT_LT(microla::wrap_two_pi(-1e30F), microla::constants::two_pi<float>());
    EXPECT_FLOAT_EQ(microla::wrap_two_pi(-std::numeric_limits<float>::denorm_min()), 0.0F);
    static_assert(microla::wrap_two_pi(1000) == 4);
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
