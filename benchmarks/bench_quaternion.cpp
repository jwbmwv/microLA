// SPDX-License-Identifier: Apache-2.0
/// @file bench_quaternion.cpp
/// @brief Quaternion operation performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// Benchmark quaternion multiplication
static void bm_quaternion_multiply(benchmark::State& state)
{
    Quaternion<float> q1(1.0F, 0.0F, 0.0F, 0.707F);
    Quaternion<float> q2(0.0F, 1.0F, 0.0F, 0.707F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q1 * q2;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_multiply);

// Benchmark spherical linear interpolation (critical for animation)
static void bm_quaternion_slerp(benchmark::State& state)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 1.0F, 0.0F), 1.5708F);
    float t = 0.5F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q1.slerp(q2, t);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_slerp);

// Benchmark quaternion-vector rotation (common in physics/graphics)
static void bm_quaternion_rotate_vector(benchmark::State& state)
{
    Quaternion<float> q = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 1.0F, 0.0F), 1.5708F);
    Vec<float, 3> v(1.0F, 0.0F, 0.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Vec<float, 3> result = q * v;
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_rotate_vector);

// Benchmark quaternion to matrix conversion
static void bm_quaternion_to_matrix(benchmark::State& state)
{
    Quaternion<float> q = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 1.0F, 0.0F), 1.5708F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto mat = q.to_matrix();
        benchmark::DoNotOptimize(mat);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_to_matrix);

// Benchmark quaternion conjugate
static void bm_quaternion_conjugate(benchmark::State& state)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q.conjugate();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_quaternion_conjugate);

// Benchmark quaternion normalization
static void bm_quaternion_normalize(benchmark::State& state)
{
    Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> result = q.normalized();
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_normalize);

// Benchmark angle between quaternions
static void bm_quaternion_angle_to(benchmark::State& state)
{
    Quaternion<float> q1 = Quaternion<float>::identity();
    Quaternion<float> q2 = Quaternion<float>::from_axis_angle(Vec<float, 3>(0.0F, 1.0F, 0.0F), 1.5708F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float angle = q1.angle_to(q2);
        benchmark::DoNotOptimize(angle);
    }
}
BENCHMARK(bm_quaternion_angle_to);

// Benchmark quaternion from euler angles
static void bm_quaternion_from_euler(benchmark::State& state)
{
    float roll = 0.1F;
    float pitch = 0.2F;
    float yaw = 0.3F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> q = Quaternion<float>::from_euler(roll, pitch, yaw);
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_from_euler);

// Benchmark quaternion from two vectors
static void bm_quaternion_from_two_vectors(benchmark::State& state)
{
    Vec<float, 3> from(1.0F, 0.0F, 0.0F);
    Vec<float, 3> to(0.0F, 1.0F, 0.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Quaternion<float> q = Quaternion<float>::from_two_vectors(from, to);
        benchmark::DoNotOptimize(q);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_quaternion_from_two_vectors);

BENCHMARK_MAIN();
