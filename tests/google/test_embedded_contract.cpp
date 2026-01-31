// test_embedded_contract.cpp - Embedded-only contract tests for no-allocation builds
#include <gtest/gtest.h>

#include "../../include/microla/compiler_features.hpp"
#include "../../include/microla/matrix.hpp"

#include <type_traits>
#include <utility>

using namespace microla;

namespace
{

template<typename Matrix, typename = void>
struct has_allocating_eigenvalues_qr : std::false_type
{
};

template<typename Matrix>
struct has_allocating_eigenvalues_qr<Matrix, std::void_t<decltype(std::declval<const Matrix&>().eigenvalues_qr())>>
    : std::true_type
{
};

struct dynamic_alloc_gate_probe
{
    MICROLA_DYNAMIC_ALLOC_ONLY(int dynamic_only() const { return 42; })
};

template<typename Type, typename = void>
struct has_dynamic_only_method : std::false_type
{
};

template<typename Type>
struct has_dynamic_only_method<Type, std::void_t<decltype(std::declval<const Type&>().dynamic_only())>> : std::true_type
{
};

}  // namespace

TEST(EmbeddedContract, EmbeddedDefaultsEnableNoDynamicAllocationContract)
{
#ifdef MICROLA_EMBEDDED
    SUCCEED() << "MICROLA_EMBEDDED is defined";
#else
    FAIL() << "MICROLA_EMBEDDED should be defined for embedded contract tests";
#endif

#ifdef MICROLA_NO_DYNAMIC_ALLOC
    SUCCEED() << "MICROLA_NO_DYNAMIC_ALLOC is defined";
#else
    FAIL() << "MICROLA_NO_DYNAMIC_ALLOC should be defined for embedded contract tests";
#endif

    EXPECT_EQ(MICROLA_HAS_DYNAMIC_ALLOC, 0);
}

TEST(EmbeddedContract, AllocatingEigenvaluesOverloadTracksDynamicAllocConfig)
{
    using Mat3f = Mat<float, 3, 3>;
#if MICROLA_HAS_DYNAMIC_ALLOC
    static_assert(has_allocating_eigenvalues_qr<Mat3f>::value,
                  "Dynamic-allocation builds should expose the convenience eigenvalues_qr overload");

    EXPECT_TRUE((has_allocating_eigenvalues_qr<Mat3f>::value));
#else
    static_assert(!has_allocating_eigenvalues_qr<Mat3f>::value,
                  "Embedded builds must not expose the allocating eigenvalues_qr convenience overload");

    EXPECT_FALSE((has_allocating_eigenvalues_qr<Mat3f>::value));
#endif
}

TEST(EmbeddedContract, DynamicAllocOnlyMacroTracksDynamicAllocConfig)
{
#if MICROLA_HAS_DYNAMIC_ALLOC
    static_assert(has_dynamic_only_method<dynamic_alloc_gate_probe>::value,
                  "MICROLA_DYNAMIC_ALLOC_ONLY must keep declarations in dynamic-allocation builds");

    EXPECT_TRUE((has_dynamic_only_method<dynamic_alloc_gate_probe>::value));
#else
    static_assert(!has_dynamic_only_method<dynamic_alloc_gate_probe>::value,
                  "MICROLA_DYNAMIC_ALLOC_ONLY must remove declarations in embedded builds");

    EXPECT_FALSE((has_dynamic_only_method<dynamic_alloc_gate_probe>::value));
#endif
}

TEST(EmbeddedContract, BufferEigenvaluesOverloadRemainsAvailable)
{
    Mat<float, 3, 3> m = Mat<float, 3, 3>::identity();
    float eigenvalues[3] = {};

    ASSERT_TRUE(m.eigenvalues_qr(eigenvalues, 3));
    for (float ev : eigenvalues)
    {
        EXPECT_NEAR(ev, 1.0F, 0.01F);
    }
}
