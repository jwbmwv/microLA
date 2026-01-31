// SPDX-License-Identifier: MIT
/// @file bench_large_matrices.cpp
/// @brief Large matrix operation benchmarks (scalability testing)
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>
#include <cstdlib>

using namespace microla;

// ===== 5x5 Matrix Operations =====

static void BM_Matrix5x5_Multiply(benchmark::State& state)
{
    SquareMat<float, 5> a, b;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }

    for (auto _ : state)
    {
        SquareMat<float, 5> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix5x5_Multiply);

static void BM_Matrix5x5_Transpose(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
            m(i, j) = static_cast<float>(i * 5 + j);

    for (auto _ : state)
    {
        SquareMat<float, 5> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix5x5_Transpose);

static void BM_Matrix5x5_VectorMultiply(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
            m(i, j) = static_cast<float>(i + j + 1);

    Vec<float, 5> v;
    for (uint32_t i = 0; i < 5; ++i)
        v[i] = static_cast<float>(i + 1);

    for (auto _ : state)
    {
        Vec<float, 5> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix5x5_VectorMultiply);

// ===== 8x8 Matrix Operations =====

static void BM_Matrix8x8_Multiply(benchmark::State& state)
{
    SquareMat<float, 8> a, b;
    for (uint32_t i = 0; i < 8; ++i)
        for (uint32_t j = 0; j < 8; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }

    for (auto _ : state)
    {
        SquareMat<float, 8> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix8x8_Multiply);

static void BM_Matrix8x8_Transpose(benchmark::State& state)
{
    SquareMat<float, 8> m;
    for (uint32_t i = 0; i < 8; ++i)
        for (uint32_t j = 0; j < 8; ++j)
            m(i, j) = static_cast<float>(i * 8 + j);

    for (auto _ : state)
    {
        SquareMat<float, 8> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix8x8_Transpose);

// ===== 10x10 Matrix Operations =====

static void BM_Matrix10x10_Multiply(benchmark::State& state)
{
    SquareMat<float, 10> a, b;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }

    for (auto _ : state)
    {
        SquareMat<float, 10> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix10x10_Multiply);

static void BM_Matrix10x10_Transpose(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
            m(i, j) = static_cast<float>(i * 10 + j);

    for (auto _ : state)
    {
        SquareMat<float, 10> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix10x10_Transpose);

static void BM_Matrix10x10_VectorMultiply(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
            m(i, j) = static_cast<float>(i + j + 1);

    Vec<float, 10> v;
    for (uint32_t i = 0; i < 10; ++i)
        v[i] = static_cast<float>(i + 1);

    for (auto _ : state)
    {
        Vec<float, 10> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Matrix10x10_VectorMultiply);

// ===== Cache Efficiency Tests =====

// Sequential access pattern (cache-friendly)
static void BM_Matrix10x10_Sequential_Access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
            m(i, j) = static_cast<float>(i * 10 + j);

    for (auto _ : state)
    {
        float sum = 0.0f;
        for (uint32_t i = 0; i < 10; ++i)
            for (uint32_t j = 0; j < 10; ++j)
                sum += m(i, j);
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(BM_Matrix10x10_Sequential_Access);

// Random access pattern (cache-unfriendly)
static void BM_Matrix10x10_Random_Access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
            m(i, j) = static_cast<float>(i * 10 + j);

    // Pre-generate random indices
    uint32_t indices[100];
    for (uint32_t k = 0; k < 100; ++k)
        indices[k] = rand() % 100;

    for (auto _ : state)
    {
        float sum = 0.0f;
        for (uint32_t k = 0; k < 100; ++k)
        {
            uint32_t idx = indices[k];
            uint32_t i = idx / 10;
            uint32_t j = idx % 10;
            sum += m(i, j);
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(BM_Matrix10x10_Random_Access);

// Column-major vs row-major access
static void BM_Matrix10x10_ColumnMajor_Access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
        for (uint32_t j = 0; j < 10; ++j)
            m(i, j) = static_cast<float>(i * 10 + j);

    for (auto _ : state)
    {
        float sum = 0.0f;
        // Access column-by-column (potentially cache-unfriendly for row-major storage)
        for (uint32_t j = 0; j < 10; ++j)
            for (uint32_t i = 0; i < 10; ++i)
                sum += m(i, j);
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(BM_Matrix10x10_ColumnMajor_Access);

// Large vector operations (cache test)
static void BM_Vector_Large_DotProduct(benchmark::State& state)
{
    Vec<float, 100> a, b;
    for (uint32_t i = 0; i < 100; ++i)
    {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
    }

    for (auto _ : state)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Vector_Large_DotProduct);

static void BM_Vector_Large_Addition(benchmark::State& state)
{
    Vec<float, 100> a, b;
    for (uint32_t i = 0; i < 100; ++i)
    {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
    }

    for (auto _ : state)
    {
        Vec<float, 100> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vector_Large_Addition);

BENCHMARK_MAIN();
