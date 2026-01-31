// test_simd_helpers.cpp - Unit tests for simd_helpers.hpp
#include <gtest/gtest.h>
#include "../../include/microla/simd_helpers.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

using namespace microla::simd;

// Test fill_float
TEST(SIMDHelpers, FillFloat)
{
    float data[100];
    std::fill(std::begin(data), std::end(data), 0.0F);

    fill_float(data, 100, 3.14F);

    for (int i = 0; i < 100; ++i)
    {
        EXPECT_FLOAT_EQ(data[i], 3.14F);
    }
}

TEST(SIMDHelpers, FillFloatPartial)
{
    float data[10] = {0};

    fill_float(data, 5, 2.71F);

    // First 5 should be filled
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_FLOAT_EQ(data[i], 2.71F);
    }

    // Rest should be zero
    for (int i = 5; i < 10; ++i)
    {
        EXPECT_FLOAT_EQ(data[i], 0.0F);
    }
}

// Test copy_n_float
TEST(SIMDHelpers, CopyNFloat)
{
    float src[100];
    float dst[100];

    for (int i = 0; i < 100; ++i)
    {
        src[i] = static_cast<float>(i);
    }

    copy_n_float(src, dst, 100);

    for (int i = 0; i < 100; ++i)
    {
        EXPECT_FLOAT_EQ(dst[i], static_cast<float>(i));
    }
}

TEST(SIMDHelpers, CopyNFloatNonAligned)
{
    float src[17];
    float dst[17];

    for (int i = 0; i < 17; ++i)
    {
        src[i] = static_cast<float>(i * 2);
    }

    copy_n_float(src, dst, 17);

    for (int i = 0; i < 17; ++i)
    {
        EXPECT_FLOAT_EQ(dst[i], static_cast<float>(i * 2));
    }
}

// Test copy4_float
TEST(SIMDHelpers, Copy4Float)
{
    float src[4] = {1.0F, 2.0F, 3.0F, 4.0F};
    float dst[4] = {0};

    copy4_float(src, dst);

    EXPECT_FLOAT_EQ(dst[0], 1.0F);
    EXPECT_FLOAT_EQ(dst[1], 2.0F);
    EXPECT_FLOAT_EQ(dst[2], 3.0F);
    EXPECT_FLOAT_EQ(dst[3], 4.0F);
}

// Test add4
TEST(SIMDHelpers, Add4)
{
    float a[4] = {1.0F, 2.0F, 3.0F, 4.0F};
    float b[4] = {5.0F, 6.0F, 7.0F, 8.0F};
    float result[4];

    add4(a, b, result);

    EXPECT_FLOAT_EQ(result[0], 6.0F);
    EXPECT_FLOAT_EQ(result[1], 8.0F);
    EXPECT_FLOAT_EQ(result[2], 10.0F);
    EXPECT_FLOAT_EQ(result[3], 12.0F);
}

// Test mul4
TEST(SIMDHelpers, Mul4)
{
    float a[4] = {2.0F, 3.0F, 4.0F, 5.0F};
    float b[4] = {1.0F, 2.0F, 3.0F, 4.0F};
    float result[4];

    mul4(a, b, result);

    EXPECT_FLOAT_EQ(result[0], 2.0F);
    EXPECT_FLOAT_EQ(result[1], 6.0F);
    EXPECT_FLOAT_EQ(result[2], 12.0F);
    EXPECT_FLOAT_EQ(result[3], 20.0F);
}

// Test fma4_accumulate
TEST(SIMDHelpers, FMA4Accumulate)
{
    float result[4] = {1.0F, 2.0F, 3.0F, 4.0F};
    float a[4] = {2.0F, 3.0F, 4.0F, 5.0F};
    float b[4] = {0.5F, 1.0F, 1.5F, 2.0F};

    fma4_accumulate(result, a, b);

    EXPECT_FLOAT_EQ(result[0], 1.0F + 2.0F * 0.5F);
    EXPECT_FLOAT_EQ(result[1], 2.0F + 3.0F * 1.0F);
    EXPECT_FLOAT_EQ(result[2], 3.0F + 4.0F * 1.5F);
    EXPECT_FLOAT_EQ(result[3], 4.0F + 5.0F * 2.0F);
}

// Test strided_copy_float
TEST(SIMDHelpers, StridedCopyFloat)
{
    float src[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    float dst[5];

    // Copy every other element
    strided_copy_float(src, 2, dst, 1, 5);

    EXPECT_FLOAT_EQ(dst[0], 0.0F);
    EXPECT_FLOAT_EQ(dst[1], 2.0F);
    EXPECT_FLOAT_EQ(dst[2], 4.0F);
    EXPECT_FLOAT_EQ(dst[3], 6.0F);
    EXPECT_FLOAT_EQ(dst[4], 8.0F);
}

// Test gather/scatter operations
TEST(SIMDHelpers, GatherNFloat)
{
    float src[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::size_t indices[5] = {1, 3, 5, 7, 9};
    float dst[5];

    gather_n_float(src, indices, dst, 5);

    EXPECT_FLOAT_EQ(dst[0], 1.0F);
    EXPECT_FLOAT_EQ(dst[1], 3.0F);
    EXPECT_FLOAT_EQ(dst[2], 5.0F);
    EXPECT_FLOAT_EQ(dst[3], 7.0F);
    EXPECT_FLOAT_EQ(dst[4], 9.0F);
}

TEST(SIMDHelpers, ScatterNFloat)
{
    float src[5] = {10.0F, 20.0F, 30.0F, 40.0F, 50.0F};
    float dst[10] = {0};
    std::size_t indices[5] = {1, 3, 5, 7, 9};

    scatter_n_float(src, dst, indices, 5);

    EXPECT_FLOAT_EQ(dst[1], 10.0F);
    EXPECT_FLOAT_EQ(dst[3], 20.0F);
    EXPECT_FLOAT_EQ(dst[5], 30.0F);
    EXPECT_FLOAT_EQ(dst[7], 40.0F);
    EXPECT_FLOAT_EQ(dst[9], 50.0F);
}

// ==================== Double Precision Tests ====================

// Test fill_double
TEST(SIMDHelpers, FillDouble)
{
    double data[100];
    std::fill(std::begin(data), std::end(data), 0.0);

    fill_double(data, 100, 2.718281828);

    for (int i = 0; i < 100; ++i)
    {
        EXPECT_DOUBLE_EQ(data[i], 2.718281828);
    }
}

TEST(SIMDHelpers, FillDoublePartial)
{
    double data[10] = {0};

    fill_double(data, 7, 1.414213562);

    // First 7 should be filled
    for (int i = 0; i < 7; ++i)
    {
        EXPECT_DOUBLE_EQ(data[i], 1.414213562);
    }

    // Rest should be zero
    for (int i = 7; i < 10; ++i)
    {
        EXPECT_DOUBLE_EQ(data[i], 0.0);
    }
}

TEST(SIMDHelpers, FillDoubleNonAligned)
{
    // Test with size not divisible by SIMD width (4 for AVX, 2 for NEON)
    double data[17];

    fill_double(data, 17, 3.14159265358979);

    for (int i = 0; i < 17; ++i)
    {
        EXPECT_DOUBLE_EQ(data[i], 3.14159265358979);
    }
}

// Test copy_n_double
TEST(SIMDHelpers, CopyNDouble)
{
    double src[100];
    double dst[100];

    for (int i = 0; i < 100; ++i)
    {
        src[i] = static_cast<double>(i) * 0.5;
    }

    copy_n_double(src, dst, 100);

    for (int i = 0; i < 100; ++i)
    {
        EXPECT_DOUBLE_EQ(dst[i], static_cast<double>(i) * 0.5);
    }
}

TEST(SIMDHelpers, CopyNDoubleNonAligned)
{
    double src[19];
    double dst[19];

    for (int i = 0; i < 19; ++i)
    {
        src[i] = static_cast<double>(i * i);
    }

    copy_n_double(src, dst, 19);

    for (int i = 0; i < 19; ++i)
    {
        EXPECT_DOUBLE_EQ(dst[i], static_cast<double>(i * i));
    }
}

TEST(SIMDHelpers, CopyNDoubleSmall)
{
    double src[3] = {1.1, 2.2, 3.3};
    double dst[3] = {0};

    copy_n_double(src, dst, 3);

    EXPECT_DOUBLE_EQ(dst[0], 1.1);
    EXPECT_DOUBLE_EQ(dst[1], 2.2);
    EXPECT_DOUBLE_EQ(dst[2], 3.3);
}

// Test double precision accuracy
TEST(SIMDHelpers, DoublePrecisionAccuracy)
{
    // Verify double precision is preserved through SIMD operations
    const double PI = 3.141592653589793238;
    const double E = 2.718281828459045235;

    double data[10];
    fill_double(data, 10, PI);

    double sum = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        sum += data[i];
    }

    EXPECT_NEAR(sum, PI * 10.0, 1e-15);
}

// Performance comparison test (not a timing test, just verify both work)
TEST(SIMDHelpers, FloatVsDoubleFill)
{
    float float_data[1000];
    double double_data[1000];

    fill_float(float_data, 1000, 1.0F);
    fill_double(double_data, 1000, 1.0);

    // Verify both completed successfully
    EXPECT_FLOAT_EQ(float_data[0], 1.0F);
    EXPECT_FLOAT_EQ(float_data[999], 1.0F);
    EXPECT_DOUBLE_EQ(double_data[0], 1.0);
    EXPECT_DOUBLE_EQ(double_data[999], 1.0);
}

// Test edge cases
TEST(SIMDHelpers, EdgeCases)
{
    // Zero-length operations
    float f_data[10] = {0};
    double d_data[10] = {0};

    fill_float(f_data, 0, 1.0F);
    fill_double(d_data, 0, 1.0);

    // Should not modify anything
    for (int i = 0; i < 10; ++i)
    {
        EXPECT_FLOAT_EQ(f_data[i], 0.0F);
        EXPECT_DOUBLE_EQ(d_data[i], 0.0);
    }
}

TEST(SIMDHelpers, SingleElement)
{
    float f_src[1] = {42.0F};
    float f_dst[1] = {0};
    double d_src[1] = {42.0};
    double d_dst[1] = {0};

    copy_n_float(f_src, f_dst, 1);
    copy_n_double(d_src, d_dst, 1);

    EXPECT_FLOAT_EQ(f_dst[0], 42.0F);
    EXPECT_DOUBLE_EQ(d_dst[0], 42.0);
}

// Test generic fill template
TEST(SIMDHelpers, GenericFillFloat)
{
    float data[50];
    fill(data, 50, 7.5F);

    for (int i = 0; i < 50; ++i)
    {
        EXPECT_FLOAT_EQ(data[i], 7.5F);
    }
}

TEST(SIMDHelpers, GenericCopyFloat)
{
    float src[50];
    float dst[50];

    for (int i = 0; i < 50; ++i)
    {
        src[i] = static_cast<float>(i + 1);
    }

    copy_n(src, dst, 50);

    for (int i = 0; i < 50; ++i)
    {
        EXPECT_FLOAT_EQ(dst[i], static_cast<float>(i + 1));
    }
}

// Test generic operations with integers
TEST(SIMDHelpers, GenericFillInt)
{
    int data[50];
    fill(data, 50, 42);

    for (int i = 0; i < 50; ++i)
    {
        EXPECT_EQ(data[i], 42);
    }
}

TEST(SIMDHelpers, GenericCopyInt)
{
    int src[50];
    int dst[50];

    for (int i = 0; i < 50; ++i)
    {
        src[i] = i * 10;
    }

    copy_n(src, dst, 50);

    for (int i = 0; i < 50; ++i)
    {
        EXPECT_EQ(dst[i], i * 10);
    }
}

// Test strided copy with generic template
TEST(SIMDHelpers, GenericStridedCopyInt)
{
    int src[20];
    int dst[10];

    for (int i = 0; i < 20; ++i)
    {
        src[i] = i;
    }

    // Copy every other element
    strided_copy(src, 2, dst, 1, 10);

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(dst[i], i * 2);
    }
}
