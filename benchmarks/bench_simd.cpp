// SPDX-License-Identifier: MIT
/// @file bench_simd.cpp
/// @brief SIMD vs non-SIMD performance comparison
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark Vec3 operations (common in 3D graphics, often SIMD-optimized)
static void BM_Vec3_DotProduct(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 2.0f, 3.0f};
    Vec<float, 3> b{4.0f, 5.0f, 6.0f};

    for (auto _ : state)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Vec3_DotProduct);

static void BM_Vec3_CrossProduct(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 0.0f, 0.0f};
    Vec<float, 3> b{0.0f, 1.0f, 0.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result = a.cross(b);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_CrossProduct);

static void BM_Vec3_Normalize(benchmark::State& state)
{
    Vec<float, 3> v{3.0f, 4.0f, 5.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result = v.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Normalize);

static void BM_Vec3_Addition(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 2.0f, 3.0f};
    Vec<float, 3> b{4.0f, 5.0f, 6.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Addition);

static void BM_Vec3_ScalarMultiply(benchmark::State& state)
{
    Vec<float, 3> v{1.0f, 2.0f, 3.0f};
    float scalar = 2.5f;

    for (auto _ : state)
    {
        Vec<float, 3> result = v * scalar;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_ScalarMultiply);

// Benchmark Vec4 operations (can benefit from 128-bit SIMD)
static void BM_Vec4_DotProduct(benchmark::State& state)
{
    Vec<float, 4> a{1.0f, 2.0f, 3.0f, 4.0f};
    Vec<float, 4> b{5.0f, 6.0f, 7.0f, 8.0f};

    for (auto _ : state)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Vec4_DotProduct);

static void BM_Vec4_Addition(benchmark::State& state)
{
    Vec<float, 4> a{1.0f, 2.0f, 3.0f, 4.0f};
    Vec<float, 4> b{5.0f, 6.0f, 7.0f, 8.0f};

    for (auto _ : state)
    {
        Vec<float, 4> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec4_Addition);

// Benchmark array of vectors (tests memory bandwidth)
static void BM_Vec3_Array_Sum(benchmark::State& state)
{
    const int N = 1000;
    Vec<float, 3> vectors[N];
    for (int i = 0; i < N; ++i)
    {
        vectors[i] = Vec<float, 3>{static_cast<float>(i), static_cast<float>(i + 1), static_cast<float>(i + 2)};
    }

    for (auto _ : state)
    {
        Vec<float, 3> sum = Vec<float, 3>::zero();
        for (int i = 0; i < N; ++i)
        {
            sum = sum + vectors[i];
        }
        benchmark::DoNotOptimize(sum);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Array_Sum);

// Benchmark quaternion operations (can use SIMD)
static void BM_Quaternion_Multiply(benchmark::State& state)
{
    Quaternion<float> q1{0.707f, 0.0f, 0.707f, 0.0f};
    Quaternion<float> q2{0.707f, 0.707f, 0.0f, 0.0f};

    for (auto _ : state)
    {
        Quaternion<float> result = q1 * q2;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Multiply);

static void BM_Quaternion_Normalize(benchmark::State& state)
{
    Quaternion<float> q{1.0f, 1.0f, 1.0f, 1.0f};

    for (auto _ : state)
    {
        Quaternion<float> result = q.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Normalize);
