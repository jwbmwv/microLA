// SPDX-License-Identifier: MIT
/// @file bench_vector_ops.cpp
/// @brief Vector operation performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>
#include <cmath>

using namespace microla;

// Benchmark vector length calculation
static void BM_Vec3_Length(benchmark::State& state)
{
    Vec<float, 3> v{3.0f, 4.0f, 5.0f};

    for (auto _ : state)
    {
        float len = v.length();
        benchmark::DoNotOptimize(len);
    }
}
BENCHMARK(BM_Vec3_Length);

// Benchmark squared length (avoids sqrt)
static void BM_Vec3_LengthSquared(benchmark::State& state)
{
    Vec<float, 3> v{3.0f, 4.0f, 5.0f};

    for (auto _ : state)
    {
        float len2 = v.length_squared();
        benchmark::DoNotOptimize(len2);
    }
}
BENCHMARK(BM_Vec3_LengthSquared);

// Benchmark distance between vectors
static void BM_Vec3_Distance(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 2.0f, 3.0f};
    Vec<float, 3> b{4.0f, 5.0f, 6.0f};

    for (auto _ : state)
    {
        float dist = a.distance(b);
        benchmark::DoNotOptimize(dist);
    }
}
BENCHMARK(BM_Vec3_Distance);

// Benchmark linear interpolation
static void BM_Vec3_Lerp(benchmark::State& state)
{
    Vec<float, 3> a{0.0f, 0.0f, 0.0f};
    Vec<float, 3> b{10.0f, 10.0f, 10.0f};
    float t = 0.5f;

    for (auto _ : state)
    {
        Vec<float, 3> result = a.lerp(b, t);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Lerp);

// Benchmark reflection
static void BM_Vec3_Reflect(benchmark::State& state)
{
    Vec<float, 3> incident{1.0f, -1.0f, 0.0f};
    Vec<float, 3> normal{0.0f, 1.0f, 0.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result = incident.reject(normal);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Reflect);

// Benchmark projection
static void BM_Vec3_Project(benchmark::State& state)
{
    Vec<float, 3> v{1.0f, 2.0f, 3.0f};
    Vec<float, 3> onto{1.0f, 0.0f, 0.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result = v.project(onto);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Project);

// Benchmark clamping
static void BM_Vec3_Clamp(benchmark::State& state)
{
    Vec<float, 3> v{5.0f, -5.0f, 15.0f};
    float min_v = 0.0f;
    float max_v = 10.0f;

    for (auto _ : state)
    {
        Vec<float, 3> result = v.clamp(min_v, max_v);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Clamp);

// Swizzle not available in current API, skipped

// Benchmark angle between vectors
static void BM_Vec3_Angle(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 0.0f, 0.0f};
    Vec<float, 3> b{0.0f, 1.0f, 0.0f};

    for (auto _ : state)
    {
        float angle = a.angle(b);
        benchmark::DoNotOptimize(angle);
    }
}
BENCHMARK(BM_Vec3_Angle);

// Benchmark component-wise operations
static void BM_Vec3_ComponentWiseMultiply(benchmark::State& state)
{
    Vec<float, 3> a{1.0f, 2.0f, 3.0f};
    Vec<float, 3> b{4.0f, 5.0f, 6.0f};

    for (auto _ : state)
    {
        Vec<float, 3> result{a[0] * b[0], a[1] * b[1], a[2] * b[2]};
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_ComponentWiseMultiply);

// Benchmark safe normalization
static void BM_Vec3_SafeNormalize(benchmark::State& state)
{
    Vec<float, 3> v{0.001f, 0.001f, 0.001f};  // Very small vector

    for (auto _ : state)
    {
        Vec<float, 3> result = v.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_SafeNormalize);

// Benchmark batch operations
static void BM_Vec3_Batch_Normalize(benchmark::State& state)
{
    const int N = 100;
    Vec<float, 3> vectors[N];
    for (int i = 0; i < N; ++i)
    {
        vectors[i] = Vec<float, 3>{static_cast<float>(i + 1), static_cast<float>(i + 2), static_cast<float>(i + 3)};
    }

    for (auto _ : state)
    {
        for (int i = 0; i < N; ++i)
        {
            vectors[i] = vectors[i].normalized();
        }
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Vec3_Batch_Normalize);
