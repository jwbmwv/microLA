// SPDX-License-Identifier: Apache-2.0
/// @file test_constants.cpp
/// @brief Tests for mathematical constants
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <gtest/gtest.h>
#include <cmath>

using namespace microla;

// ===== Constants Tests =====

TEST(ConstantsTest, Pi)
{
    float pi_f = constants::pi<float>();
    double pi_d = constants::pi<double>();

    EXPECT_NEAR(pi_f, 3.14159265358979323846F, 1e-6F);
    EXPECT_NEAR(pi_d, 3.14159265358979323846, 1e-14);
}

TEST(ConstantsTest, TwoPi)
{
    float two_pi = constants::two_pi<float>();
    EXPECT_NEAR(two_pi, 2.0F * constants::pi<float>(), 1e-6F);
}

TEST(ConstantsTest, HalfPi)
{
    float half_pi = constants::half_pi<float>();
    EXPECT_NEAR(half_pi, constants::pi<float>() / 2.0F, 1e-6F);
}

TEST(ConstantsTest, E)
{
    float e_f = constants::e<float>();
    double e_d = constants::e<double>();

    EXPECT_NEAR(e_f, 2.71828182845904523536F, 1e-6F);
    EXPECT_NEAR(e_d, 2.71828182845904523536, 1e-14);
}

TEST(ConstantsTest, Sqrt2)
{
    float sqrt2 = constants::sqrt2<float>();
    EXPECT_NEAR(sqrt2, std::sqrt(2.0F), 1e-6F);
}

TEST(ConstantsTest, Sqrt3)
{
    float sqrt3 = constants::sqrt3<float>();
    EXPECT_NEAR(sqrt3, std::sqrt(3.0F), 1e-6F);
}

TEST(ConstantsTest, GoldenRatio)
{
    float phi = constants::golden_ratio<float>();
    EXPECT_NEAR(phi, (1.0F + std::sqrt(5.0F)) / 2.0F, 1e-6F);
}

TEST(ConstantsTest, DegToRad)
{
    float deg_to_rad = constants::deg_to_rad<float>();
    EXPECT_NEAR(deg_to_rad, constants::pi<float>() / 180.0F, 1e-6F);
}

TEST(ConstantsTest, RadToDeg)
{
    float rad_to_deg = constants::rad_to_deg<float>();
    EXPECT_NEAR(rad_to_deg, 180.0F / constants::pi<float>(), 1e-5F);
}

TEST(ConstantsTest, ConversionRoundTrip)
{
    float angle_deg = 45.0F;
    float angle_rad = angle_deg * constants::deg_to_rad<float>();
    float angle_deg_back = angle_rad * constants::rad_to_deg<float>();

    EXPECT_NEAR(angle_deg_back, angle_deg, 1e-5F);
}

// ===== Type-specific Constants =====

TEST(ConstantsTypeTest, FloatConstants)
{
    EXPECT_TRUE((std::is_same<decltype(constants::pi<float>()), float>::value));
    EXPECT_TRUE(std::isfinite(constants::pi<float>()));
}

TEST(ConstantsTypeTest, DoubleConstants)
{
    EXPECT_TRUE((std::is_same<decltype(constants::pi<double>()), double>::value));
    EXPECT_TRUE(std::isfinite(constants::pi<double>()));
}

// ===== Usage Examples =====

TEST(ConstantsUsageTest, AngleConversions)
{
    // 90 degrees to radians
    float angle_deg = 90.0F;
    float angle_rad = angle_deg * constants::deg_to_rad<float>();
    EXPECT_NEAR(angle_rad, constants::half_pi<float>(), 1e-6F);

    // 180 degrees to radians
    float angle_deg_180 = 180.0F;
    float angle_rad_180 = angle_deg_180 * constants::deg_to_rad<float>();
    EXPECT_NEAR(angle_rad_180, constants::pi<float>(), 1e-6F);
}

TEST(ConstantsUsageTest, CircleCircumference)
{
    float radius = 5.0F;
    float circumference = constants::two_pi<float>() * radius;
    EXPECT_NEAR(circumference, 31.41593F, 1e-4F);
}

TEST(ConstantsUsageTest, IsoscelesRightTriangle)
{
    float leg = 1.0F;
    float hypotenuse = leg * constants::sqrt2<float>();
    EXPECT_NEAR(hypotenuse, std::sqrt(2.0F), 1e-6F);
}
