// test_compiler_features.cpp - Unit tests for compiler_features.hpp
#include <gtest/gtest.h>
#include "../../include/microla/compiler_features.hpp"
#include "../../include/microla/vector.hpp"
#include "../../include/microla/matrix.hpp"
#include "../../include/microla/quaternion.hpp"
#include "../../include/microla/fast_math.hpp"
#include "../../include/microla/safe_math.hpp"
#include <cstdint>
#include <type_traits>

using namespace microla;

// Helper for nodiscard test: define at file scope so attribute applies to a function
[[nodiscard]] int nodiscard_helper()
{
    return 42;
}

// Test ROM/Flash placement macros
TEST(CompilerFeatures, ROMFlashPlacementMacros)
{
    // Verify macros are defined (compilation test)
    static constexpr MICROLA_RODATA float rodata_array[] = {1.0F, 2.0F, 3.0F};
    static constexpr MICROLA_FLASH float flash_array[] = {4.0F, 5.0F, 6.0F};

    EXPECT_EQ(rodata_array[0], 1.0F);
    EXPECT_EQ(flash_array[0], 4.0F);
}

// Test cache line size detection
TEST(CompilerFeatures, CacheLineSizeDetection)
{
    // Verify MICROLA_CACHE_LINE_SIZE is defined and has a reasonable value
    EXPECT_GE(MICROLA_CACHE_LINE_SIZE, 16);
    EXPECT_LE(MICROLA_CACHE_LINE_SIZE, 256);
}

// Test cache-aligned allocation
TEST(CompilerFeatures, CacheAlignedAllocation)
{
    struct MICROLA_CACHE_ALIGNED CacheAlignedData
    {
        float data[16];
    };

    CacheAlignedData aligned_data;
    aligned_data.data[0] = 1.0F;

    // Verify alignment
    auto addr = reinterpret_cast<std::uintptr_t>(&aligned_data);
    EXPECT_EQ(addr % MICROLA_CACHE_LINE_SIZE, 0U);
}

// Test DMA alignment
TEST(CompilerFeatures, DMAAlignedAllocation)
{
    struct MICROLA_DMA_ALIGNED DMABuffer
    {
        float data[256];
    };

    DMABuffer buffer;
    buffer.data[0] = 1.0F;

    // Verify DMA alignment (typically 32 or 64 bytes)
    auto addr = reinterpret_cast<std::uintptr_t>(&buffer);
    EXPECT_EQ(addr % 32, 0U);
}

// Test memory barrier macros (compilation test)
TEST(CompilerFeatures, MemoryBarrierCompilation)
{
    volatile int shared_data = 0;

    // Write with memory barrier
    MICROLA_COMPILER_BARRIER();
    shared_data = 42;
    MICROLA_MEMORY_BARRIER();

    // Read with memory barrier
    MICROLA_MEMORY_BARRIER();
    int local_copy = shared_data;
    MICROLA_COMPILER_BARRIER();

    EXPECT_EQ(local_copy, 42);
}

// Test ISR-safe marker (compilation test)
TEST(CompilerFeatures, ISRSafeMarker)
{
    // Function marked ISR-safe should compile
    auto isr_safe_func = []() MICROLA_ISR_SAFE
    {
        volatile int dummy = 0;
        dummy = 1;
        (void)dummy;
    };

    isr_safe_func();
    SUCCEED();
}

// Test timing markers (documentation markers)
TEST(CompilerFeatures, TimingMarkers)
{
    // These are documentation markers, verify they compile
    auto critical_section = []() MICROLA_TIMING_CRITICAL
    {
        Vec<float, 3> v(1.0F, 2.0F, 3.0F);
        return v.dot(v);
    };

    float result = critical_section();
    EXPECT_GT(result, 0.0F);
}

// Test constexpr support under C++17 baseline
TEST(CompilerFeatures, ConstexprSupport)
{
    constexpr auto test_constexpr = []() constexpr
    {
        int sum = 0;
        for (int i = 0; i < 10; ++i)
        {
            sum += i;
        }
        return sum;
    };

    constexpr int result = test_constexpr();
    EXPECT_EQ(result, 45);
}

// Test [[nodiscard]]
TEST(CompilerFeatures, NodiscardAttribute)
{
    int result = nodiscard_helper();
    EXPECT_EQ(result, 42);
}

// Test embedded system defaults
#ifdef MICROLA_EMBEDDED
TEST(CompilerFeatures, EmbeddedDefaults)
{
    // When MICROLA_EMBEDDED is defined, verify defaults are set
#ifdef MICROLA_NO_EXCEPTIONS
    SUCCEED() << "MICROLA_NO_EXCEPTIONS is defined";
#else
    FAIL() << "MICROLA_NO_EXCEPTIONS should be defined for embedded builds";
#endif

#ifdef MICROLA_NO_RTTI
    SUCCEED() << "MICROLA_NO_RTTI is defined";
#else
    FAIL() << "MICROLA_NO_RTTI should be defined for embedded builds";
#endif

#ifdef MICROLA_NO_DYNAMIC_ALLOC
    SUCCEED() << "MICROLA_NO_DYNAMIC_ALLOC is defined";
#else
    FAIL() << "MICROLA_NO_DYNAMIC_ALLOC should be defined for embedded builds";
#endif

    // Check stack size limit is conservative for embedded
    EXPECT_LE(MICROLA_STACK_SIZE_LIMIT, 2048);
}
#endif

// Test stack size limits
TEST(CompilerFeatures, StackSizeLimits)
{
    // Verify MICROLA_STACK_SIZE_LIMIT is defined
    EXPECT_GT(MICROLA_STACK_SIZE_LIMIT, 0);

#ifdef MICROLA_EMBEDDED
    EXPECT_LE(MICROLA_STACK_SIZE_LIMIT, 2048);
#else
    EXPECT_GE(MICROLA_STACK_SIZE_LIMIT, 4096);
#endif
}

// Test type traits
TEST(CompilerFeatures, TypeTraits)
{
    EXPECT_TRUE(std::is_floating_point_v<float>);
    EXPECT_TRUE(std::is_floating_point_v<double>);
    EXPECT_FALSE(std::is_floating_point_v<int>);

    EXPECT_TRUE(std::is_integral_v<int>);
    EXPECT_TRUE(std::is_integral_v<uint32_t>);
    EXPECT_FALSE(std::is_integral_v<float>);

    EXPECT_TRUE(std::is_arithmetic_v<float>);
    EXPECT_TRUE(std::is_arithmetic_v<int>);
    EXPECT_FALSE(std::is_arithmetic_v<void*>);
}

// Test if constexpr
TEST(CompilerFeatures, IfConstexpr)
{
    auto test_if_constexpr = [](auto value)
    {
        if constexpr (std::is_floating_point_v<decltype(value)>)
        {
            return static_cast<int>(value + 0.5);
        }
        else
        {
            return static_cast<int>(value);
        }
    };

    EXPECT_EQ(test_if_constexpr(3.7F), 4);
    EXPECT_EQ(test_if_constexpr(5), 5);
}

// Test C++20 features (if available)
#if __cplusplus >= 202002L
TEST(CompilerFeatures, Cpp20Features)
{
    // Test MICROLA_CONSTEXPR20
    constexpr auto test_constexpr20 = []() MICROLA_CONSTEXPR20
    {
        int value = 10;
        return value * 2;
    };

    constexpr int result = test_constexpr20();
    EXPECT_EQ(result, 20);

    // Test MICROLA_CONSTEVAL
    auto test_consteval = []() MICROLA_CONSTEVAL { return 42; };

    // MICROLA_CONSTEVAL should force compile-time evaluation
    constexpr int eval_result = test_consteval();
    EXPECT_EQ(eval_result, 42);

    // Test MICROLA_CONSTEVAL usage inside compiler_features.hpp
    constexpr bool stack_limit_valid = microla::detail::stack_size_limit_is_valid();
    EXPECT_TRUE(stack_limit_valid);

    // Test MICROLA_CONSTEXPR20 adoption in fast_math
    constexpr float abs_result = microla::fast::abs(-3.5F);
    constexpr float sqrt_result = microla::fast::sqrt<>(4.0F);
    EXPECT_FLOAT_EQ(abs_result, 3.5F);
    EXPECT_GT(sqrt_result, 1.9F);
    EXPECT_LT(sqrt_result, 2.1F);
}
#endif

// Test C++26 trig constexpr features (if available)
#if __cplusplus >= 202600L
TEST(CompilerFeatures, Cpp26ConstexprTrig)
{
    constexpr Vec<float, 2> v(3.0F, 4.0F);
    constexpr float len = v.length();
    static_assert(len > 4.99F && len < 5.01F, "Vec::length should be constexpr with C++26 trig support");

    constexpr auto rot = Mat<float, 3, 3>::rotation_z(0.25F);
    static_assert(rot(0, 0) > 0.0F, "rotation_z should be constexpr with C++26 trig support");

    constexpr Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
    constexpr float q_norm = q.norm();
    static_assert(q_norm > 5.4F && q_norm < 5.6F, "Quaternion::norm should be constexpr with C++26 trig support");

    EXPECT_GT(len, 4.99F);
    EXPECT_LT(len, 5.01F);
    EXPECT_GT(q_norm, 5.4F);
    EXPECT_LT(q_norm, 5.6F);
}
#endif

TEST(CompilerFeatures, UnreachableBackedSafeMathPaths)
{
    // Exercise safe_math finite/NaN helpers; these now include explicit
    // unreachable fallbacks for unsupported floating-point widths.
    EXPECT_TRUE(microla::safe::is_finite(1.0F));
    EXPECT_TRUE(microla::safe::is_finite(1.0));
    EXPECT_FALSE(microla::safe::is_nan(1.0F));
    EXPECT_FALSE(microla::safe::is_nan(1.0));
}

// Test C++23 features (if available)
#if __cplusplus >= 202302L
TEST(CompilerFeatures, Cpp23Features)
{
    // Test MICROLA_CONSTEXPR23
    constexpr auto test_constexpr23 = []() MICROLA_CONSTEXPR23 { return "C++23 constexpr"; };

    constexpr const char* result = test_constexpr23();
    EXPECT_STREQ(result, "C++23 constexpr");
}
#endif

// Test standard layout guarantees
TEST(CompilerFeatures, StandardLayoutGuarantees)
{
    // Verify Vec is standard layout
    // Use bool variables to avoid macro expansion issues with commas
    bool vec_is_standard_layout = std::is_standard_layout_v<Vec<float, 3>>;
    bool mat_is_standard_layout = std::is_standard_layout_v<Mat<float, 4, 4>>;
    bool quat_is_standard_layout = std::is_standard_layout_v<Quaternion<float>>;

    EXPECT_TRUE(vec_is_standard_layout);
    EXPECT_TRUE(mat_is_standard_layout);
    EXPECT_TRUE(quat_is_standard_layout);
}

// Test POD compatibility
TEST(CompilerFeatures, PODCompatibility)
{
    // Verify types are trivially copyable (safe for memcpy, DMA)
    // Use bool variables to avoid macro expansion issues with commas
    bool vec_is_pod = std::is_trivially_copyable<Vec<float, 3>>::value;
    bool mat_is_pod = std::is_trivially_copyable<Mat<float, 4, 4>>::value;
    bool quat_is_pod = std::is_trivially_copyable<Quaternion<float>>::value;

    EXPECT_TRUE(vec_is_pod);
    EXPECT_TRUE(mat_is_pod);
    EXPECT_TRUE(quat_is_pod);
}

// Test alignment compatibility
TEST(CompilerFeatures, AlignmentCompatibility)
{
    // Verify 16-byte alignment for SIMD
    EXPECT_GE(alignof(Vec<float, 3>), 16U);
    EXPECT_GE(alignof(Vec<float, 4>), 16U);
    EXPECT_GE(alignof(Mat<float, 4, 4>), 16U);
    EXPECT_GE(alignof(Quaternion<float>), 16U);
}
