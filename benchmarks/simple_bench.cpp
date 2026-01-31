// SPDX-License-Identifier: Apache-2.0
/// @file simple_bench.cpp
/// @brief Simple benchmark runner without external dependencies
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#define MICROLA_LINEAR_HEADER_ONLY
#include "../include/microla/microla.hpp"
#include <math.h>

#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>

using namespace microla;
using Vec3f = Vec<float, 3>;
using namespace std::chrono;

// Simple benchmark harness
template<typename Func>
double benchmark(const std::string& name, Func&& func, int iterations = 1000000)
{
    // Warmup
    for (int i = 0; i < 100; ++i)
    {
        func();
    }

    auto start = high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i)
    {
        func();
    }
    auto end = high_resolution_clock::now();

    auto duration = duration_cast<nanoseconds>(end - start).count();
    double avg_ns = static_cast<double>(duration) / iterations;

    std::cout << std::left << std::setw(40) << name << std::right << std::setw(12) << std::fixed << std::setprecision(2)
              << avg_ns << " ns/op" << std::endl;

    return avg_ns;
}

int main()
{
    std::cout << "\n=== MicroLA Performance Benchmarks ===\n" << std::endl;

    // Vector Operations
    std::cout << "--- Vector Operations ---" << std::endl;

    Vec3f v1{3.0f, 4.0f, 5.0f};
    Vec3f v2{1.0f, 2.0f, 3.0f};
    volatile float result_f = NAN;
    Vec3f result_v;

    benchmark("Vec3f: Length", [&]() { result_f = v1.length(); });

    benchmark("Vec3f: Length Squared", [&]() { result_f = v1.length_squared(); });

    benchmark("Vec3f: Dot Product", [&]() { result_f = v1.dot(v2); });

    benchmark("Vec3f: Cross Product", [&]() { result_v = v1.cross(v2); });

    benchmark("Vec3f: Normalize", [&]() { result_v = v1.normalized(); });

    benchmark("Vec3f: Addition", [&]() { result_v = v1 + v2; });

    benchmark("Vec3f: Scalar Multiply", [&]() { result_v = v1 * 2.5F; });

    benchmark("Vec3f: Distance", [&]() { result_f = v1.distance(v2); });

    benchmark("Vec3f: Lerp", [&]() { result_v = v1.lerp(v2, 0.5F); });

    // Matrix Operations
    std::cout << "\n--- Matrix Operations ---" << std::endl;

    SquareMat<float, 3> m3x3_a = SquareMat<float, 3>::identity();
    SquareMat<float, 3> m3x3_b;
    for (uint32_t i = 0; i < 3; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            m3x3_b(i, j) = static_cast<float>(i + j + 1);
        }
    }

    SquareMat<float, 4> m4x4_a = SquareMat<float, 4>::identity();
    SquareMat<float, 4> m4x4_b;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m4x4_b(i, j) = static_cast<float>(i + j);
        }
    }

    SquareMat<float, 3> result_m3;
    SquareMat<float, 4> result_m4;

    benchmark("Mat3x3: Multiply", [&]() { result_m3 = m3x3_a * m3x3_b; });

    benchmark("Mat4x4: Multiply", [&]() { result_m4 = m4x4_a * m4x4_b; });

    benchmark("Mat3x3: Transpose", [&]() { result_m3 = m3x3_b.transpose(); });

    benchmark("Mat4x4: Transpose", [&]() { result_m4 = m4x4_b.transpose(); });

    benchmark("Mat3x3: Determinant", [&]() { result_f = m3x3_b.determinant(); });

    benchmark("Mat4x4: Determinant", [&]() { result_f = m4x4_b.determinant(); });

    benchmark("Mat3x3: Inverse", [&]() { result_m3 = m3x3_b.inverse(); });

    benchmark("Mat4x4: Inverse", [&]() { result_m4 = m4x4_b.inverse(); });

    Vec<float, 4> v4{1.0f, 2.0f, 3.0f, 4.0f};
    Vec<float, 4> result_v4;

    benchmark("Mat4x4 * Vec4", [&]() { result_v4 = m4x4_b * v4; });

    // Quaternion Operations
    std::cout << "\n--- Quaternion Operations ---" << std::endl;

    Quaternion<float> q1 = Quaternion<float>::from_axis_angle(Vec3f{0.0f, 1.0f, 0.0f}, constants::pi<float> / 4.0f);
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec3f{1.0f, 0.0f, 0.0f}, constants::pi<float> / 6.0f);
    Quaternion<float> result_q;

    benchmark("Quaternion: Multiply", [&]() { result_q = q1 * q2; });

    benchmark("Quaternion: Normalize", [&]() { result_q = q1.normalized(); });

    benchmark("Quaternion: Conjugate", [&]() { result_q = q1.conjugate(); });

    benchmark("Quaternion: To Matrix", [&]() { result_m3 = q1.to_matrix(); });

    benchmark("Quaternion: SLERP", [&]() { result_q = q1.slerp(q2, 0.5F); });

    // Transformation Operations
    std::cout << "\n--- Transformation Operations ---" << std::endl;

    benchmark("Create Rotation Matrix (runtime)",
              [&]()
              {
                  // rotation_x is defined for 3x3 matrices; embed into 4x4
                  auto r3 = SquareMat<float, 3>::rotation_x(constants::pi<float> / 4.0F);
                  result_m4 = SquareMat<float, 4>::identity();
                  for (std::uint32_t i = 0; i < 3; ++i)
                  {
                      for (std::uint32_t j = 0; j < 3; ++j)
                      {
                          result_m4(i, j) = r3(i, j);
                      }
                  }
              });

    benchmark("Create Translation Matrix",
              [&]()
              {
                  result_m4 = SquareMat<float, 4>::identity();
                  result_m4(0, 3) = 1.0f;
                  result_m4(1, 3) = 2.0f;
                  result_m4(2, 3) = 3.0f;
              });

    benchmark("Create Scale Matrix",
              [&]()
              {
                  result_m4 = SquareMat<float, 4>::identity();
                  result_m4(0, 0) = 2.0f;
                  result_m4(1, 1) = 2.0f;
                  result_m4(2, 2) = 2.0f;
              });

    // Matrix Decompositions
    std::cout << "\n--- Matrix Decompositions ---" << std::endl;

    SquareMat<float, 3> decomp_mat;
    decomp_mat(0, 0) = 4.0F;
    decomp_mat(0, 1) = 3.0F;
    decomp_mat(0, 2) = 2.0F;
    decomp_mat(1, 0) = 3.0F;
    decomp_mat(1, 1) = 5.0F;
    decomp_mat(1, 2) = 1.0F;
    decomp_mat(2, 0) = 2.0F;
    decomp_mat(2, 1) = 1.0F;
    decomp_mat(2, 2) = 6.0F;

    benchmark("LU Decomposition", [&]() { auto lu = decomp_mat.lu(); }, 100000);

    benchmark("QR Decomposition", [&]() { auto qr = decomp_mat.qr(); }, 100000);

    benchmark("Cholesky Decomposition", [&]() { auto chol = decomp_mat.cholesky(); }, 100000);

    // Constexpr vs Runtime
    std::cout << "\n--- Compile-Time vs Runtime ---" << std::endl;

    benchmark("Identity (constexpr)",
              [&]()
              {
                  constexpr auto id = SquareMat<float, 4>::identity();
                  result_m4 = id;
              });

    benchmark("Identity (runtime)", [&]() { result_m4 = SquareMat<float, 4>::identity(); });

    benchmark("Zero Vec (constexpr)",
              [&]()
              {
                  auto zero = Vec3f::zero();
                  result_v = zero;
              });

    benchmark("Zero Vec (runtime)", [&]() { result_v = Vec3f::zero(); });

    std::cout << "\n=== Benchmarks Complete ===\n" << std::endl;

    return 0;
}
