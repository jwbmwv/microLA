// SPDX-License-Identifier: Apache-2.0
/// @file bench_large_matrices.cpp
/// @brief Large matrix operation benchmarks (scalability testing)
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>
#include <cstdlib>

using namespace microla;

// ===== 5x5 Matrix Operations =====

static void bm_matrix5x5_multiply(benchmark::State& state)
{
    SquareMat<float, 5> a;
    SquareMat<float, 5> b;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 5> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix5x5_multiply);

static void bm_matrix5x5_transpose(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            m(i, j) = static_cast<float>(i * 5 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 5> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix5x5_transpose);

static void bm_matrix5x5_vector_multiply(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            m(i, j) = static_cast<float>(i + j + 1);
        }
    }

    Vec<float, 5> v;
    for (uint32_t i = 0; i < 5; ++i)
    {
        v[i] = static_cast<float>(i + 1);
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 5> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix5x5_vector_multiply);

// ===== 8x8 Matrix Operations =====

static void bm_matrix8x8_multiply(benchmark::State& state)
{
    SquareMat<float, 8> a;
    SquareMat<float, 8> b;
    for (uint32_t i = 0; i < 8; ++i)
    {
        for (uint32_t j = 0; j < 8; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 8> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix8x8_multiply);

static void bm_matrix8x8_transpose(benchmark::State& state)
{
    SquareMat<float, 8> m;
    for (uint32_t i = 0; i < 8; ++i)
    {
        for (uint32_t j = 0; j < 8; ++j)
        {
            m(i, j) = static_cast<float>(i * 8 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 8> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix8x8_transpose);

// ===== 10x10 Matrix Operations =====

static void bm_matrix10x10_multiply(benchmark::State& state)
{
    SquareMat<float, 10> a;
    SquareMat<float, 10> b;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            a(i, j) = static_cast<float>(i + j + 1);
            b(i, j) = static_cast<float>(i * 2 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 10> result = a * b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix10x10_multiply);

static void bm_matrix10x10_transpose(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            m(i, j) = static_cast<float>(i * 10 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 10> result = m.transpose();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix10x10_transpose);

static void bm_matrix10x10_vector_multiply(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            m(i, j) = static_cast<float>(i + j + 1);
        }
    }

    Vec<float, 10> v;
    for (uint32_t i = 0; i < 10; ++i)
    {
        v[i] = static_cast<float>(i + 1);
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 10> result = m * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_matrix10x10_vector_multiply);

// ===== Cache Efficiency Tests =====

// Sequential access pattern (cache-friendly)
static void bm_matrix10x10_sequential_access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            m(i, j) = static_cast<float>(i * 10 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float sum = 0.0F;
        for (uint32_t i = 0; i < 10; ++i)
        {
            for (uint32_t j = 0; j < 10; ++j)
            {
                sum += m(i, j);
            }
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_matrix10x10_sequential_access);

// Random access pattern (cache-unfriendly)
static void bm_matrix10x10_random_access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            m(i, j) = static_cast<float>(i * 10 + j);
        }
    }

    // Pre-generate random indices
    uint32_t indices[100];
    for (unsigned int& indice : indices)
    {
        indice = rand() % 100;
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float sum = 0.0F;
        for (unsigned int idx : indices)
        {
            uint32_t i = idx / 10;
            uint32_t j = idx % 10;
            sum += m(i, j);
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_matrix10x10_random_access);

// Column-major vs row-major access
static void bm_matrix10x10_column_major_access(benchmark::State& state)
{
    SquareMat<float, 10> m;
    for (uint32_t i = 0; i < 10; ++i)
    {
        for (uint32_t j = 0; j < 10; ++j)
        {
            m(i, j) = static_cast<float>(i * 10 + j);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float sum = 0.0F;
        // Access column-by-column (potentially cache-unfriendly for row-major storage)
        for (uint32_t j = 0; j < 10; ++j)
        {
            for (uint32_t i = 0; i < 10; ++i)
            {
                sum += m(i, j);
            }
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_matrix10x10_column_major_access);

// Large vector operations (cache test)
static void bm_vector_large_dot_product(benchmark::State& state)
{
    Vec<float, 100> a;
    Vec<float, 100> b;
    for (uint32_t i = 0; i < 100; ++i)
    {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_vector_large_dot_product);

static void bm_vector_large_addition(benchmark::State& state)
{
    Vec<float, 100> a;
    Vec<float, 100> b;
    for (uint32_t i = 0; i < 100; ++i)
    {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 100> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vector_large_addition);

BENCHMARK_MAIN();
