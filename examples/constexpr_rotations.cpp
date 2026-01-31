// SPDX-License-Identifier: Apache-2.0
/// @file constexpr_rotations.cpp
/// @brief Example demonstrating compile-time rotation matrices
/// @details Shows how to create rotation matrices at compile time for different C++ standards
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>

using namespace microla;

// Helper to print matrix
template<typename T, std::size_t N>
void print_matrix(const char* name, const Mat<T, N, N>& mat)
{
    std::cout << name << ":\n";
    for (std::size_t i = 0; i < N; ++i)
    {
        std::cout << "  [";
        for (std::size_t j = 0; j < N; ++j)
        {
            std::cout << std::setw(8) << std::fixed << std::setprecision(4) << mat[i][j];
            if (j < N - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "]\n";
    }
    std::cout << "\n";
}

auto main() -> int
{
    std::cout << "=== MicroLA Compile-Time Rotation Examples ===\n\n";

    // ========================================
    // C++17-C++23: Special Angles Only
    // ========================================
    std::cout << "--- C++17+: Special Angle Rotations ---\n";
    std::cout << "These compile to constants!\n\n";

    // Identity matrix - true compile-time constant
    constexpr auto identity2 = Mat<float, 2, 2>::identity();
    constexpr auto identity3 = Mat<float, 3, 3>::identity();
    constexpr auto identity4 = Mat<float, 4, 4>::identity();

    std::cout << "Identity matrices (compile-time constants):\n";
    print_matrix("2×2 Identity", identity2);
    print_matrix("3×3 Identity", identity3);
    print_matrix("4×4 Identity", identity4);

    // Runtime rotations (computed once, still efficient)
    auto r90z = Mat<float, 3, 3>::rotation_z(deg_to_rad(90.0F));
    auto r180x = Mat<float, 3, 3>::rotation_x(deg_to_rad(180.0F));
    auto r270y = Mat<float, 3, 3>::rotation_y(deg_to_rad(270.0F));

    print_matrix("90° rotation around Z", r90z);
    print_matrix("180° rotation around X", r180x);
    print_matrix("270° rotation around Y", r270y);

    // Test with vector
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = r90z * v;

    std::cout << "Original vector: (" << v[0] << ", " << v[1] << ", " << v[2] << ")\n";
    std::cout << "After 90° Z rotation: (" << rotated[0] << ", " << rotated[1] << ", " << rotated[2] << ")\n";
    std::cout << "(Should be approximately (0, 1, 0))\n\n";

    // 2D rotation
    auto r2_d = Mat<float, 2, 2>::rotation(deg_to_rad(90.0F));
    print_matrix("90° rotation in 2D", r2_d);

    // ========================================
    // Runtime: Arbitrary Angles
    // ========================================
    std::cout << "--- Runtime: Arbitrary Angle Rotations ---\n";
    std::cout << "These are computed at runtime (uses sin/cos):\n\n";

    auto r_37_5 = Mat<float, 3, 3>::rotation_z(deg_to_rad(37.5F));
    auto r_pi_4 = Mat<float, 3, 3>::rotation_x(constants::pi<float>() / 4.0F);

    print_matrix("37.5° rotation around Z (runtime)", r_37_5);
    print_matrix("45° (π/4) rotation around X (runtime)", r_pi_4);

    // ========================================
    // C++26: Any Angle at Compile Time
    // ========================================
    std::cout << "--- C++26 Preview: Arbitrary Angles at Compile Time ---\n";

#if __cplusplus >= 202600L
    // C++26: This works at compile time!
    constexpr auto R_arbitrary = Mat<float, 3, 3>::rotation_z(1.2345F);
    print_matrix("1.2345 rad rotation around Z (C++26 compile-time!)", R_arbitrary);

    constexpr auto R_37deg = Mat<float, 3, 3>::rotation_y(deg_to_rad(37.5F));
    print_matrix("37.5° rotation around Y (C++26 compile-time!)", R_37deg);

    std::cout << "✓ Running with C++26: Full constexpr trigonometry available!\n\n";
#else
    std::cout << "Currently compiled with C++";
    if (__cplusplus == 202002L)
    {
        std::cout << "20";
    }
    else if (__cplusplus == 201703L)
    {
        std::cout << "17";
    }
    else
    {
        std::cout << " (unknown)";
    }
    std::cout << "\nC++26 required for compile-time arbitrary angle rotations.\n";
    std::cout << "Recompile with -std=c++2c when available.\n\n";
#endif

    // ========================================
    // Practical Example: Sensor Orientation
    // ========================================
    std::cout << "--- Practical Example: Fixed Sensor Mounting ---\n";
    std::cout << "IMU mounted 90° rotated on PCB (pre-computed transform):\n\n";

    // Sensor coordinate system rotated 90° from body frame
    static const auto sensor_to_body = Mat<float, 3, 3>::rotation_z(deg_to_rad(90.0F));

    // Simulated IMU reading in sensor frame
    Vec<float, 3> imu_reading(1.0F, 0.5F, 0.2F);

    // Transform to body frame (matrix already in memory, no computation!)
    Vec<float, 3> body_frame = sensor_to_body * imu_reading;

    std::cout << "IMU reading (sensor frame): (" << imu_reading[0] << ", " << imu_reading[1] << ", " << imu_reading[2]
              << ")\n";
    std::cout << "Transformed (body frame):   (" << body_frame[0] << ", " << body_frame[1] << ", " << body_frame[2]
              << ")\n";
    std::cout << "\n✓ Transformation matrix pre-computed - minimal runtime overhead!\n\n";

    // ========================================
    // Performance Note
    // ========================================
    std::cout << "=== Performance Notes ===\n";
    std::cout << "Pre-computed rotations:\n";
    std::cout << "  - Computed once at initialization\n";
    std::cout << "  - Stored as constants in memory\n";
    std::cout << "  - No repeated sin/cos calls\n";
    std::cout << "  - Perfect for embedded systems with fixed orientations\n\n";

    std::cout << "Runtime rotations:\n";
    std::cout << "  - Required for dynamic angles\n";
    std::cout << "  - Uses hardware FPU when available\n";
    std::cout << "  - SIMD optimized on supported platforms\n\n";

    return 0;
}
