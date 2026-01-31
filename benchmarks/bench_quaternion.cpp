// SPDX-License-Identifier: MIT
/// @file bench_quaternion.cpp
/// @brief Quaternion operation performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark quaternion multiplication
static void BM_Quaternion_Multiply(benchmark::State& state)
{
    Quaternion<float> q1(1.0f, 0.0f, 0.0f, 0.707f);
    Quaternion<float> q2(0.0f, 1.0f, 0.0f, 0.707f);

    for (auto _ : state)
    {
        Quaternion<float> result = q1 * q2;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Multiply);

// Benchmark spherical linear interpolation (critical for animation)
static void BM_Quaternion_Slerp(benchmark::State& state)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0f, 1.0f, 0.0f), 1.5708f);
    float t = 0.5f;

    for (auto _ : state)
    {
        Quaternion<float> result = q1.slerp(q2, t);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Slerp);

// Benchmark quaternion-vector rotation (common in physics/graphics)
static void BM_Quaternion_RotateVector(benchmark::State& state)
{
    Quaternion<float> q = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0f, 1.0f, 0.0f), 1.5708f);
    Vec<float, 3> v(1.0f, 0.0f, 0.0f);

    for (auto _ : state)
    {
        Vec<float, 3> result = q * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_RotateVector);

// Benchmark quaternion to matrix conversion
static void BM_Quaternion_ToMatrix(benchmark::State& state)
{
    Quaternion<float> q = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0f, 1.0f, 0.0f), 1.5708f);

    for (auto _ : state)
    {
        auto mat = q.to_matrix();
        benchmark::DoNotOptimize(mat);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_ToMatrix);

// Benchmark quaternion conjugate
static void BM_Quaternion_Conjugate(benchmark::State& state)
{
    Quaternion<float> q(1.0f, 2.0f, 3.0f, 4.0f);

    for (auto _ : state)
    {
        Quaternion<float> result = q.conjugate();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Quaternion_Conjugate);

// Benchmark quaternion normalization
static void BM_Quaternion_Normalize(benchmark::State& state)
{
    Quaternion<float> q(1.0f, 2.0f, 3.0f, 4.0f);

    for (auto _ : state)
    {
        Quaternion<float> result = q.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_Normalize);

// Benchmark angle between quaternions
static void BM_Quaternion_AngleTo(benchmark::State& state)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0f, 1.0f, 0.0f), 1.5708f);

    for (auto _ : state)
    {
        float angle = q1.angle_to(q2);
        benchmark::DoNotOptimize(angle);
    }
}
BENCHMARK(BM_Quaternion_AngleTo);

// Benchmark quaternion from euler angles
static void BM_Quaternion_FromEuler(benchmark::State& state)
{
    float roll = 0.1f, pitch = 0.2f, yaw = 0.3f;

    for (auto _ : state)
    {
        Quaternion<float> q = Quaternion<float>::from_euler(roll, pitch, yaw);
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_FromEuler);

// Benchmark quaternion from two vectors
static void BM_Quaternion_FromTwoVectors(benchmark::State& state)
{
    Vec<float, 3> from(1.0f, 0.0f, 0.0f);
    Vec<float, 3> to(0.0f, 1.0f, 0.0f);

    for (auto _ : state)
    {
        Quaternion<float> q = Quaternion<float>::from_two_vectors(from, to);
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Quaternion_FromTwoVectors);

BENCHMARK_MAIN();
