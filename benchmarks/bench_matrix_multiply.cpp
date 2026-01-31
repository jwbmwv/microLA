// SPDX-License-Identifier: Apache-2.0
/// @file bench_matrix_multiply.cpp
/// @brief Matrix multiplication performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark 3x3 matrix multiplication (common in graphics)
static void bm_matrix3x3_multiply_float(benchmark::State& state)
{
    SquareMat<float, 3> a = SquareMat<float, 3>::identity();
    SquareMat<float, 3> b;
    b(0, 0) = 2.0F;
    b(0, 1) = 0.0F;
    b(0, 2) = 1.0F;
    b(1, 0) = 0.0F;
    b(1, 1) = 2.0F;
    b(1, 2) = 0.0F;
    b(2, 0) = 1.0F;
    b(2, 1) = 0.0F;
    b(2, 2) = 2.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 3> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix3x3_multiply_float);

// Benchmark 4x4 matrix multiplication (common for 3D transforms)
static void bm_matrix4x4_multiply_float(benchmark::State& state)
{
    SquareMat<float, 4> a = SquareMat<float, 4>::identity();
    SquareMat<float, 4> b;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            b(i, j) = static_cast<float>(i + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 4> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix4x4_multiply_float);

// Benchmark double precision
static void bm_matrix4x4_multiply_double(benchmark::State& state)
{
    SquareMat<double, 4> a = SquareMat<double, 4>::identity();
    SquareMat<double, 4> b;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            b(i, j) = static_cast<double>(i + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<double, 4> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix4x4_multiply_double);

// Benchmark chain multiplication (A * B * C)
static void bm_matrix3x3_chain_multiply(benchmark::State& state)
{
    SquareMat<float, 3> a = SquareMat<float, 3>::identity();
    SquareMat<float, 3> b = SquareMat<float, 3>::rotation_z(1.5708F);  // 90 degrees in radians
    SquareMat<float, 3> c;
    c(0, 0) = 2.0F;
    c(1, 1) = 2.0F;
    c(2, 2) = 2.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 3> result = a * b * c;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix3x3_chain_multiply);

// Benchmark matrix-vector multiplication
static void bm_matrix4x4_vector_multiply(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m(i, j) = static_cast<float>(i + j + 1);
        }
    }
    Vec<float, 4> v{1.0F, 2.0F, 3.0F, 4.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 4> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix4x4_vector_multiply);

// Benchmark transpose
static void bm_matrix4x4_transpose(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m(i, j) = static_cast<float>(i * 4 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 4> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix4x4_transpose);

// Benchmark determinant calculation
static void bm_matrix3x3_determinant(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 2.0F;
    m(0, 1) = 3.0F;
    m(0, 2) = 1.0F;
    m(1, 0) = 4.0F;
    m(1, 1) = 1.0F;
    m(1, 2) = 5.0F;
    m(2, 0) = 3.0F;
    m(2, 1) = 2.0F;
    m(2, 2) = 1.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(bm_matrix3x3_determinant);

// Benchmark inverse calculation
static void bm_matrix3x3_inverse(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 2.0F;
    m(0, 1) = 3.0F;
    m(0, 2) = 1.0F;
    m(1, 0) = 4.0F;
    m(1, 1) = 1.0F;
    m(1, 2) = 5.0F;
    m(2, 0) = 3.0F;
    m(2, 1) = 2.0F;
    m(2, 2) = 1.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 3> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix3x3_inverse);
