// SPDX-License-Identifier: MIT
/// @file bench_constexpr.cpp
/// @brief Constexpr vs runtime initialization benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark compile-time identity matrix usage
static void BM_Identity_Constexpr(benchmark::State& state)
{
    for (auto _ : state)
    {
        auto identity = SquareMat<float, 4>::identity();
        benchmark::DoNotOptimize(identity);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Identity_Constexpr);

// Benchmark runtime identity matrix creation
static void BM_Identity_Runtime(benchmark::State& state)
{
    for (auto _ : state)
    {
        SquareMat<float, 4> identity = SquareMat<float, 4>::identity();
        benchmark::DoNotOptimize(identity);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Identity_Runtime);

// Benchmark compile-time zero vector
static void BM_Zero_Vector_Constexpr(benchmark::State& state)
{
    for (auto _ : state)
    {
        auto zero = Vec<float, 3>::zero();
        benchmark::DoNotOptimize(zero);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Zero_Vector_Constexpr);

// Benchmark runtime zero vector
static void BM_Zero_Vector_Runtime(benchmark::State& state)
{
    for (auto _ : state)
    {
        Vec<float, 3> zero = Vec<float, 3>::zero();
        benchmark::DoNotOptimize(zero);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Zero_Vector_Runtime);

// Benchmark compile-time rotation matrix (special angles)
static void BM_Rotation_CompileTime(benchmark::State& state)
{
    for (auto _ : state)
    {
        auto rot90 = SquareMat<float, 2>::rotation(1.5708f);  // 90 degrees
        benchmark::DoNotOptimize(rot90);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Rotation_CompileTime);

// Benchmark runtime rotation matrix
static void BM_Rotation_Runtime(benchmark::State& state)
{
    const float angle = 1.5707963f;  // 90 degrees in radians

    for (auto _ : state)
    {
        auto rot = SquareMat<float, 2>::rotation(angle);
        benchmark::DoNotOptimize(rot);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Rotation_Runtime);

// Benchmark using pre-computed lookups
static void BM_Rotation_Lookup_Table(benchmark::State& state)
{
    static const SquareMat<float, 2> rotations[] = {
        SquareMat<float, 2>::rotation(0.0f), SquareMat<float, 2>::rotation(1.5708f),
        SquareMat<float, 2>::rotation(3.14159f), SquareMat<float, 2>::rotation(4.71239f)};

    int idx = 0;
    for (auto _ : state)
    {
        auto rot = rotations[idx % 4];
        benchmark::DoNotOptimize(rot);
        idx++;
    }
}
BENCHMARK(BM_Rotation_Lookup_Table);

// Benchmark identity quaternion
static void BM_Quaternion_Identity_Constexpr(benchmark::State& state)
{
    for (auto _ : state)
    {
        auto q = Quaternion<float>::identity();
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Identity_Constexpr);

// Benchmark mathematical constants usage
static void BM_Constants_CompileTime(benchmark::State& state)
{
    for (auto _ : state)
    {
        float pi = constants::pi<float>();
        float two_pi = constants::two_pi<float>();
        float result = pi * two_pi;
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Constants_CompileTime);
