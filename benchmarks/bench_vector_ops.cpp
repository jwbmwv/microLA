// SPDX-License-Identifier: Apache-2.0
/// @file bench_vector_ops.cpp
/// @brief Vector operation performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>
#include <cmath>

using namespace microla;

// Benchmark vector length calculation
static void bm_vec3_length(benchmark::State& state)
{
    Vec<float, 3> v{3.0F, 4.0F, 5.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float len = v.length();
        benchmark::DoNotOptimize(len);
    }
}
BENCHMARK(bm_vec3_length);

// Benchmark squared length (avoids sqrt)
static void bm_vec3_length_squared(benchmark::State& state)
{
    Vec<float, 3> v{3.0F, 4.0F, 5.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float len2 = v.length_squared();
        benchmark::DoNotOptimize(len2);
    }
}
BENCHMARK(bm_vec3_length_squared);

// Benchmark distance between vectors
static void bm_vec3_distance(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 2.0F, 3.0F};
    Vec<float, 3> b{4.0F, 5.0F, 6.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float dist = a.distance(b);
        benchmark::DoNotOptimize(dist);
    }
}
BENCHMARK(bm_vec3_distance);

// Benchmark linear interpolation
static void bm_vec3_lerp(benchmark::State& state)
{
    Vec<float, 3> a{0.0F, 0.0F, 0.0F};
    Vec<float, 3> b{10.0F, 10.0F, 10.0F};
    float t = 0.5F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = a.lerp(b, t);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_lerp);

// Benchmark reflection
static void bm_vec3_reflect(benchmark::State& state)
{
    Vec<float, 3> incident{1.0F, -1.0F, 0.0F};
    Vec<float, 3> normal{0.0F, 1.0F, 0.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = incident.reject(normal);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_reflect);

// Benchmark projection
static void bm_vec3_project(benchmark::State& state)
{
    Vec<float, 3> v{1.0F, 2.0F, 3.0F};
    Vec<float, 3> onto{1.0F, 0.0F, 0.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = v.project(onto);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_project);

// Benchmark clamping
static void bm_vec3_clamp(benchmark::State& state)
{
    Vec<float, 3> v{5.0F, -5.0F, 15.0F};
    float min_v = 0.0F;
    float max_v = 10.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = v.clamp(min_v, max_v);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_clamp);

// Swizzle not available in current API, skipped

// Benchmark angle between vectors
static void bm_vec3_angle(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 0.0F, 0.0F};
    Vec<float, 3> b{0.0F, 1.0F, 0.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float angle = a.angle(b);
        benchmark::DoNotOptimize(angle);
    }
}
BENCHMARK(bm_vec3_angle);

// Benchmark component-wise operations
static void bm_vec3_component_wise_multiply(benchmark::State& state)
{
    Vec<float, 3> a{1.0F, 2.0F, 3.0F};
    Vec<float, 3> b{4.0F, 5.0F, 6.0F};

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result{a[0] * b[0], a[1] * b[1], a[2] * b[2]};
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_component_wise_multiply);

// Benchmark safe normalization
static void bm_vec3_safe_normalize(benchmark::State& state)
{
    Vec<float, 3> v{0.001F, 0.001F, 0.001F};  // Very small vector

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = v.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_safe_normalize);

// Benchmark batch operations
static void bm_vec3_batch_normalize(benchmark::State& state)
{
    const int n = 100;
    Vec<float, 3> vectors[n];
    for (int i = 0; i < n; ++i)
    {
        vectors[i] = Vec<float, 3>{static_cast<float>(i + 1), static_cast<float>(i + 2), static_cast<float>(i + 3)};
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        for (auto& vector : vectors)
        {
            vector = vector.normalized();
        }
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_vec3_batch_normalize);
