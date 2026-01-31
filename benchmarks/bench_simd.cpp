// SPDX-License-Identifier: Apache-2.0
/// @file bench_simd.cpp
/// @brief SIMD vs non-SIMD performance comparison
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark Vec3 operations (common in 3D graphics, often SIMD-optimized)
static void bm_vec3_dot_product(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 2.0F, 3.0F};
    Vec<float, 3> b{4.0F, 5.0F, 6.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_vec3_dot_product);

static void bm_vec3_cross_product(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 0.0F, 0.0F};
    Vec<float, 3> b{0.0F, 1.0F, 0.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = a.cross(b);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_cross_product);

static void bm_vec3_normalize(benchmark::State& state)
{
    Vec<float, 3> v{3.0F, 4.0F, 5.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = v.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_normalize);

static void bm_vec3_addition(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 2.0F, 3.0F};
    Vec<float, 3> b{4.0F, 5.0F, 6.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_addition);

static void bm_vec3_scalar_multiply(benchmark::State& state)
{
    Vec<float, 3> v{1.0F, 2.0F, 3.0F};
    float scalar = 2.5F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = v * scalar;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_scalar_multiply);

// Benchmark Vec4 operations (can benefit from 128-bit SIMD)
static void bm_vec4_dot_product(benchmark::State& state)
{
    Vec<float, 4> a{1.0F, 2.0F, 3.0F, 4.0F};
    Vec<float, 4> b{5.0F, 6.0F, 7.0F, 8.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float result = a.dot(b);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_vec4_dot_product);

static void bm_vec4_addition(benchmark::State& state)
{
    Vec<float, 4> a{1.0F, 2.0F, 3.0F, 4.0F};
    Vec<float, 4> b{5.0F, 6.0F, 7.0F, 8.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 4> result = a + b;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec4_addition);

// Benchmark array of vectors (tests memory bandwidth)
static void bm_vec3_array_sum(benchmark::State& state)
{
    const int n = 1000;
    Vec<float, 3> vectors[n];
    for (int i = 0; i < n; ++i)
    {
        vectors[i] = Vec<float, 3>{static_cast<float>(i), static_cast<float>(i + 1), static_cast<float>(i + 2)};
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> sum = Vec<float, 3>::zero();
        for (const auto& vector : vectors)
        {
            sum = sum + vector;
        }
        benchmark::DoNotOptimize(sum);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_array_sum);

// Benchmark quaternion operations (can use SIMD)
static void bm_quaternion_multiply(benchmark::State& state)
{
    Quaternion<float> q1{0.707F, 0.0F, 0.707F, 0.0F};
    Quaternion<float> q2{0.707F, 0.707F, 0.0F, 0.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q1 * q2;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_multiply);

static void bm_quaternion_normalize(benchmark::State& state)
{
    Quaternion<float> q{1.0F, 1.0F, 1.0F, 1.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_normalize);
