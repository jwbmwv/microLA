// SPDX-License-Identifier: MIT
/// @file bench_matrix_multiply.cpp
/// @brief Matrix multiplication performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark 3x3 matrix multiplication (common in graphics)
static void BM_Matrix3x3_Multiply_Float(benchmark::State& state)
{
    SquareMat<float, 3> a = SquareMat<float, 3>::identity();
    SquareMat<float, 3> b;
    b(0, 0) = 2.0f;
    b(0, 1) = 0.0f;
    b(0, 2) = 1.0f;
    b(1, 0) = 0.0f;
    b(1, 1) = 2.0f;
    b(1, 2) = 0.0f;
    b(2, 0) = 1.0f;
    b(2, 1) = 0.0f;
    b(2, 2) = 2.0f;

    for (auto _ : state)
    {
        SquareMat<float, 3> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix3x3_Multiply_Float);

// Benchmark 4x4 matrix multiplication (common for 3D transforms)
static void BM_Matrix4x4_Multiply_Float(benchmark::State& state)
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

    for (auto _ : state)
    {
        SquareMat<float, 4> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix4x4_Multiply_Float);

// Benchmark double precision
static void BM_Matrix4x4_Multiply_Double(benchmark::State& state)
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

    for (auto _ : state)
    {
        SquareMat<double, 4> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix4x4_Multiply_Double);

// Benchmark chain multiplication (A * B * C)
static void BM_Matrix3x3_Chain_Multiply(benchmark::State& state)
{
    SquareMat<float, 3> a = SquareMat<float, 3>::identity();
    SquareMat<float, 3> b = SquareMat<float, 3>::rotation_z(1.5708f);  // 90 degrees in radians
    SquareMat<float, 3> c;
    c(0, 0) = 2.0f;
    c(1, 1) = 2.0f;
    c(2, 2) = 2.0f;

    for (auto _ : state)
    {
        SquareMat<float, 3> result = a * b * c;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix3x3_Chain_Multiply);

// Benchmark matrix-vector multiplication
static void BM_Matrix4x4_Vector_Multiply(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m(i, j) = static_cast<float>(i + j + 1);
        }
    }
    Vec<float, 4> v{1.0f, 2.0f, 3.0f, 4.0f};

    for (auto _ : state)
    {
        Vec<float, 4> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix4x4_Vector_Multiply);

// Benchmark transpose
static void BM_Matrix4x4_Transpose(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m(i, j) = static_cast<float>(i * 4 + j);
        }
    }

    for (auto _ : state)
    {
        SquareMat<float, 4> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix4x4_Transpose);

// Benchmark determinant calculation
static void BM_Matrix3x3_Determinant(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 2.0f;
    m(0, 1) = 3.0f;
    m(0, 2) = 1.0f;
    m(1, 0) = 4.0f;
    m(1, 1) = 1.0f;
    m(1, 2) = 5.0f;
    m(2, 0) = 3.0f;
    m(2, 1) = 2.0f;
    m(2, 2) = 1.0f;

    for (auto _ : state)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(BM_Matrix3x3_Determinant);

// Benchmark inverse calculation
static void BM_Matrix3x3_Inverse(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 2.0f;
    m(0, 1) = 3.0f;
    m(0, 2) = 1.0f;
    m(1, 0) = 4.0f;
    m(1, 1) = 1.0f;
    m(1, 2) = 5.0f;
    m(2, 0) = 3.0f;
    m(2, 1) = 2.0f;
    m(2, 2) = 1.0f;

    for (auto _ : state)
    {
        SquareMat<float, 3> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix3x3_Inverse);
