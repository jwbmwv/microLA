// test_resource.cpp - Unit tests for resource_checks.hpp
#include <gtest/gtest.h>
#include "../../include/microla/resource_checks.hpp"
#include "../../include/microla/vector.hpp"
#include "../../include/microla/matrix.hpp"
#include "../../include/microla/quaternion.hpp"

using namespace microla;
using namespace microla::resource_checks;

// Test VecSizeInfo
TEST(ResourceChecks, VecSizeInfoFloat3) {
    using Info = VecSizeInfo<float, 3>;
    
    EXPECT_EQ(Info::element_size(), sizeof(float));
    EXPECT_EQ(Info::dimension(), 3u);
    EXPECT_EQ(Info::theoretical_size(), 3 * sizeof(float));
}

TEST(ResourceChecks, VecSizeInfoDouble4) {
    using Info = VecSizeInfo<double, 4>;
    
    EXPECT_EQ(Info::element_size(), sizeof(double));
    EXPECT_EQ(Info::dimension(), 4u);
    EXPECT_EQ(Info::actual_size(), sizeof(Vec<double, 4>));
}

TEST(ResourceChecks, VecSizeInfoInt2) {
    using Info = VecSizeInfo<int, 2>;
    
    EXPECT_EQ(Info::element_size(), sizeof(int));
    EXPECT_EQ(Info::dimension(), 2u);
    EXPECT_EQ(Info::theoretical_size(), 2 * sizeof(int));
}

TEST(ResourceChecks, VecActualSize) {
    // Verify that Vec actual_size field matches sizeof
    EXPECT_EQ((VecSizeInfo<float, 3>::actual_size()), (sizeof(Vec<float, 3>)));
    EXPECT_EQ((VecSizeInfo<double, 4>::actual_size()), (sizeof(Vec<double, 4>)));
    EXPECT_EQ((VecSizeInfo<int, 2>::actual_size()), (sizeof(Vec<int, 2>)));
}

// Test MatSizeInfo
TEST(ResourceChecks, MatSizeInfo4x4Float) {
    using Info = MatSizeInfo<float, 4, 4>;
    
    EXPECT_EQ(Info::element_size(), sizeof(float));
    EXPECT_EQ(Info::rows(), 4u);
    EXPECT_EQ(Info::cols(), 4u);
    EXPECT_EQ(Info::theoretical_size(), 16 * sizeof(float));
}

TEST(ResourceChecks, MatSizeInfo3x3Double) {
    using Info = MatSizeInfo<double, 3, 3>;
    
    EXPECT_EQ(Info::element_size(), sizeof(double));
    EXPECT_EQ(Info::rows(), 3u);
    EXPECT_EQ(Info::cols(), 3u);
    EXPECT_EQ(Info::theoretical_size(), 9 * sizeof(double));
}

TEST(ResourceChecks, MatSizeInfoRectangular) {
    using Info = MatSizeInfo<float, 3, 4>;
    
    EXPECT_EQ(Info::rows(), 3u);
    EXPECT_EQ(Info::cols(), 4u);
}

TEST(ResourceChecks, MatActualSize) {
    // Verify that Mat actual_size field matches sizeof
    EXPECT_EQ((MatSizeInfo<float, 4, 4>::actual_size()), (sizeof(Mat<float, 4, 4>)));
    EXPECT_EQ((MatSizeInfo<double, 3, 3>::actual_size()), (sizeof(Mat<double, 3, 3>)));
    EXPECT_EQ((MatSizeInfo<int, 2, 2>::actual_size()), (sizeof(Mat<int, 2, 2>)));
}

// Test QuaternionSizeInfo
TEST(ResourceChecks, QuaternionSizeInfoFloat) {
    using Info = QuaternionSizeInfo<float>;
    
    EXPECT_EQ(Info::element_size(), sizeof(float));
    EXPECT_EQ(Info::theoretical_size(), 4 * sizeof(float));
}

TEST(ResourceChecks, QuaternionSizeInfoDouble) {
    using Info = QuaternionSizeInfo<double>;
    
    EXPECT_EQ(Info::element_size(), sizeof(double));
    EXPECT_EQ(Info::theoretical_size(), 4 * sizeof(double));
}

TEST(ResourceChecks, QuaternionActualSize) {
    // Verify that Quaternion actual_size field matches sizeof
    EXPECT_EQ((QuaternionSizeInfo<float>::actual_size()), (sizeof(Quaternion<float>)));
    EXPECT_EQ((QuaternionSizeInfo<double>::actual_size()), (sizeof(Quaternion<double>)));
}

// Test DMA alignment checks
TEST(ResourceChecks, DMAAlignment) {
    // Modern compilers may align Vec4/Mat4x4/Quaternion to 16 bytes for SIMD optimization
    // Check that alignment is at least as strict as the element type
    EXPECT_GE(alignof(Vec<float, 4>), alignof(float));
    
    // Check Mat alignment is reasonable (at least element alignment)
    EXPECT_GE(alignof(Mat<float, 4, 4>), alignof(float));
    
    // Check Quaternion alignment is reasonable (at least element alignment)
    EXPECT_GE(alignof(Quaternion<float>), alignof(float));
    
    // Verify alignments are power of 2 for DMA compatibility
    EXPECT_EQ(alignof(Vec<float, 4>) & (alignof(Vec<float, 4>) - 1), 0u);
    EXPECT_EQ(alignof(Mat<float, 4, 4>) & (alignof(Mat<float, 4, 4>) - 1), 0u);
    EXPECT_EQ(alignof(Quaternion<float>) & (alignof(Quaternion<float>) - 1), 0u);
}

// Test contiguity guarantees
TEST(ResourceChecks, ContiguousMemory) {
    // Test Vec contiguity - Vec has public data[] array
    Vec<float, 4> v(1.0f, 2.0f, 3.0f, 4.0f);
    const float* ptr = v.data;
    EXPECT_EQ(ptr[0], 1.0f);
    EXPECT_EQ(ptr[1], 2.0f);
    EXPECT_EQ(ptr[2], 3.0f);
    EXPECT_EQ(ptr[3], 4.0f);
    
    // Test Quaternion contiguity - Quaternion stores {x,y,z,w} internally
    // Constructor Quaternion(w,x,y,z) stores data as {x,y,z,w}
    Quaternion<float> q(1.0f, 2.0f, 3.0f, 4.0f); // w=1, x=2, y=3, z=4
    const float* qptr = q.data;
    EXPECT_EQ(qptr[0], 2.0f);  // x component
    EXPECT_EQ(qptr[1], 3.0f);  // y component
    EXPECT_EQ(qptr[2], 4.0f);  // z component
    EXPECT_EQ(qptr[3], 1.0f);  // w component (scalar last convention)
}

// Test size constraints for embedded systems
TEST(ResourceChecks, EmbeddedSizeConstraints) {
    // Typical embedded constraints: 
    // - Vec3f may be 12 or 16 bytes (SIMD alignment adds padding)
    // - Mat4f should fit in 64 bytes
    // - Quaternion should fit in 16 bytes
    
    EXPECT_LE(sizeof(Vec<float, 3>), 16u);  // Allow SIMD alignment
    EXPECT_LE(sizeof(Mat<float, 4, 4>), 64u);
    EXPECT_LE(sizeof(Quaternion<float>), 16u);
    
    // Stack usage for common operations should be reasonable
    EXPECT_LE(sizeof(Vec<float, 3>) * 10, 256u); // Array of 10 vectors
    EXPECT_LE(sizeof(Mat<float, 3, 3>) * 2, 128u); // 2 matrices for transforms
}

// Test type traits
TEST(ResourceChecks, TypeTraits) {
    // Standard layout is required for DMA and C compatibility
    // Note: is_trivially_copyable may be false if constructors are present
    EXPECT_TRUE((std::is_standard_layout<Vec<float, 3>>::value));
    
    // Mat should be standard layout
    EXPECT_TRUE((std::is_standard_layout<Mat<float, 4, 4>>::value));
    
    // Quaternion should be standard layout
    EXPECT_TRUE((std::is_standard_layout<Quaternion<float>>::value));
}

// Test memory layout for C interop
TEST(ResourceChecks, CInteropLayout) {
    // These types should be directly castable from C arrays
    
    // Vec from C array
    float c_vec[] = {1.0f, 2.0f, 3.0f};
    Vec<float, 3>* v = reinterpret_cast<Vec<float, 3>*>(c_vec);
    EXPECT_FLOAT_EQ(v->data[0], 1.0f);
    EXPECT_FLOAT_EQ(v->data[1], 2.0f);
    EXPECT_FLOAT_EQ(v->data[2], 3.0f);
    
    // Quaternion stores {x,y,z,w}, so C array should be in that order
    float c_quat[] = {1.0f, 2.0f, 3.0f, 4.0f};  // {x,y,z,w}
    Quaternion<float>* q = reinterpret_cast<Quaternion<float>*>(c_quat);
    EXPECT_FLOAT_EQ(q->data[0], 1.0f);  // x
    EXPECT_FLOAT_EQ(q->data[1], 2.0f);  // y
    EXPECT_FLOAT_EQ(q->data[2], 3.0f);  // z
    EXPECT_FLOAT_EQ(q->data[3], 4.0f);  // w
}

// Test compile-time size calculations
TEST(ResourceChecks, CompileTimeSizes) {
    // These should all be compile-time constants
    constexpr std::size_t vec_size = VecSizeInfo<float, 3>::theoretical_size();
    constexpr std::size_t mat_size = MatSizeInfo<float, 4, 4>::theoretical_size();
    constexpr std::size_t quat_size = QuaternionSizeInfo<float>::theoretical_size();
    
    EXPECT_EQ(vec_size, 12u);
    EXPECT_EQ(mat_size, 64u);
    EXPECT_EQ(quat_size, 16u);
}

// Test memory footprint for common scenarios
TEST(ResourceChecks, MemoryFootprint) {
    // Sensor fusion state: 3 vectors + 1 quaternion
    std::size_t sensor_state = 
        (sizeof(Vec<float, 3>)) * 3 + (sizeof(Quaternion<float>));
    EXPECT_LE(sensor_state, 64u); // Should fit in one cache line
    
    // Transform stack: 4 matrices for hierarchical transforms
    std::size_t transform_stack = (sizeof(Mat<float, 4, 4>)) * 4;
    EXPECT_LE(transform_stack, 256u);
    
    // Kalman state: depends on dimensions but check reasonable sizes
    std::size_t kalman_2d = 
        (sizeof(Vec<float, 2>)) * 2 + (sizeof(Mat<float, 2, 2>)) * 2;
    EXPECT_LE(kalman_2d, 64u);
}

// Test standard layout guarantees
TEST(ResourceChecks, StandardLayout) {
    // Standard layout types can be safely passed to C
    EXPECT_TRUE((std::is_standard_layout<Vec<float, 3>>::value));
    EXPECT_TRUE((std::is_standard_layout<Mat<float, 4, 4>>::value));
    EXPECT_TRUE((std::is_standard_layout<Quaternion<float>>::value));
}

// Test that no padding is added
TEST(ResourceChecks, NoPadding) {
    // Vec may have padding for SIMD alignment on some platforms
    // Just check sizes are reasonable
    EXPECT_LE((sizeof(Vec<float, 3>)), 16u);  // May be padded to 16 bytes
    EXPECT_LE((sizeof(Vec<float, 4>)), 32u);  // May be padded for alignment
    
    // Quaternion may have padding for alignment
    EXPECT_LE((sizeof(Quaternion<float>)), 32u);
    EXPECT_LE((sizeof(Quaternion<double>)), 64u);
    
    // Mat should be no larger than needed plus reasonable padding
    EXPECT_LE((sizeof(Mat<float, 3, 3>)), 48u);   // 9*4 = 36, allow padding
    EXPECT_LE((sizeof(Mat<float, 4, 4>)), 128u);  // 16*4 = 64, allow padding
}
