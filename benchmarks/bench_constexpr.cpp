// SPDX-License-Identifier: Apache-2.0
/// @file bench_constexpr.cpp
/// @brief Constexpr vs runtime initialization benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark compile-time identity matrix usage
static void bm_identity_constexpr(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto identity = SquareMat<float, 4>::identity();
        benchmark::DoNotOptimize(identity);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_identity_constexpr);

// Benchmark runtime identity matrix creation
static void bm_identity_runtime(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 4> identity = SquareMat<float, 4>::identity();
        benchmark::DoNotOptimize(identity);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_identity_runtime);

// Benchmark compile-time zero vector
static void bm_zero_vector_constexpr(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto zero = Vec<float, 3>::zero();
        benchmark::DoNotOptimize(zero);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_zero_vector_constexpr);

// Benchmark runtime zero vector
static void bm_zero_vector_runtime(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> zero = Vec<float, 3>::zero();
        benchmark::DoNotOptimize(zero);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_zero_vector_runtime);

// Benchmark compile-time rotation matrix (special angles)
static void bm_rotation_compile_time(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto rot90 = SquareMat<float, 2>::rotation(1.5708F);  // 90 degrees
        benchmark::DoNotOptimize(rot90);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_rotation_compile_time);

// Benchmark runtime rotation matrix
static void bm_rotation_runtime(benchmark::State& state)
{
    const float angle = 1.5707963F;  // 90 degrees in radians

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto rot = SquareMat<float, 2>::rotation(angle);
        benchmark::DoNotOptimize(rot);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_rotation_runtime);

// Benchmark using pre-computed lookups
static void bm_rotation_lookup_table(benchmark::State& state)
{
    static const SquareMat<float, 2> rotations[] = {
        SquareMat<float, 2>::rotation(0.0F), SquareMat<float, 2>::rotation(1.5708F),
        SquareMat<float, 2>::rotation(3.14159F), SquareMat<float, 2>::rotation(4.71239F)};

    int idx = 0;
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto rot = rotations[idx % 4];
        benchmark::DoNotOptimize(rot);
        idx++;
    }
}
BENCHMARK(bm_rotation_lookup_table);

// Benchmark identity quaternion
static void bm_quaternion_identity_constexpr(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto q = Quaternion<float>::identity();
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_identity_constexpr);

// Benchmark mathematical constants usage
static void bm_constants_compile_time(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float pi = constants::pi<float>();
        float two_pi = constants::two_pi<float>();
        float result = pi * two_pi;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_constants_compile_time);
