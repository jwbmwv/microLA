// SPDX-License-Identifier: MIT
/// @file bench_algorithms.cpp
/// @brief Numerical algorithm performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// ===== QR Decomposition Benchmarks =====

static void BM_QR_3x3(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 12.0f;
    m(0, 1) = -51.0f;
    m(0, 2) = 4.0f;
    m(1, 0) = 6.0f;
    m(1, 1) = 167.0f;
    m(1, 2) = -68.0f;
    m(2, 0) = -4.0f;
    m(2, 1) = 24.0f;
    m(2, 2) = -41.0f;

    for (auto _ : state)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_QR_3x3);

static void BM_QR_4x4(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
        for (uint32_t j = 0; j < 4; ++j)
            m(i, j) = static_cast<float>(i * 4 + j + 1);

    for (auto _ : state)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_QR_4x4);

static void BM_QR_5x5(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
            m(i, j) = static_cast<float>(i * 5 + j + 1);

    for (auto _ : state)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_QR_5x5);

// ===== Eigenvalue Computation Benchmarks =====

static void BM_Eigenvalues_3x3_Identity(benchmark::State& state)
{
    SquareMat<float, 3> m = SquareMat<float, 3>::identity();

    for (auto _ : state)
    {
        auto eigenvalues = m.eigenvaluesQR(100);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Eigenvalues_3x3_Identity);

static void BM_Eigenvalues_3x3_Symmetric(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 4.0f;
    m(0, 1) = 1.0f;
    m(0, 2) = 1.0f;
    m(1, 0) = 1.0f;
    m(1, 1) = 3.0f;
    m(1, 2) = 2.0f;
    m(2, 0) = 1.0f;
    m(2, 1) = 2.0f;
    m(2, 2) = 5.0f;

    for (auto _ : state)
    {
        auto eigenvalues = m.eigenvaluesQR(100);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Eigenvalues_3x3_Symmetric);

// ===== Determinant Benchmarks =====

static void BM_Determinant_4x4(benchmark::State& state)
{
    SquareMat<float, 4> m;
    m(0, 0) = 1.0f;
    m(0, 1) = 2.0f;
    m(0, 2) = 3.0f;
    m(0, 3) = 4.0f;
    m(1, 0) = 5.0f;
    m(1, 1) = 6.0f;
    m(1, 2) = 7.0f;
    m(1, 3) = 8.0f;
    m(2, 0) = 9.0f;
    m(2, 1) = 2.0f;
    m(2, 2) = 3.0f;
    m(2, 3) = 1.0f;
    m(3, 0) = 4.0f;
    m(3, 1) = 5.0f;
    m(3, 2) = 6.0f;
    m(3, 3) = 7.0f;

    for (auto _ : state)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(BM_Determinant_4x4);

static void BM_Determinant_5x5(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
            m(i, j) = static_cast<float>((i + j) % 5 + 1);

    for (auto _ : state)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(BM_Determinant_5x5);

// ===== Matrix Inverse Benchmarks =====

static void BM_Inverse_4x4(benchmark::State& state)
{
    // Use a known invertible 4x4 matrix (det ≠ 0)
    SquareMat<float, 4> m;
    m(0, 0) = 4.0f;
    m(0, 1) = 3.0f;
    m(0, 2) = 0.0f;
    m(0, 3) = 0.0f;
    m(1, 0) = 3.0f;
    m(1, 1) = 2.0f;
    m(1, 2) = 0.0f;
    m(1, 3) = 0.0f;
    m(2, 0) = 0.0f;
    m(2, 1) = 0.0f;
    m(2, 2) = 1.0f;
    m(2, 3) = 0.5f;
    m(3, 0) = 0.0f;
    m(3, 1) = 0.0f;
    m(3, 2) = 0.5f;
    m(3, 3) = 1.0f;

    for (auto _ : state)
    {
        SquareMat<float, 4> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Inverse_4x4);

static void BM_Inverse_5x5(benchmark::State& state)
{
    // Use diagonal-dominant matrix (guaranteed invertible)
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
        for (uint32_t j = 0; j < 5; ++j)
            m(i, j) = static_cast<float>((i == j) ? 10.0f : (i + j) % 3 + 0.5f);

    for (auto _ : state)
    {
        SquareMat<float, 5> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_Inverse_5x5);

// ===== Combined Algorithm Workflow =====

static void BM_QR_Then_Eigenvalues_3x3(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 4.0f;
    m(0, 1) = 1.0f;
    m(0, 2) = 1.0f;
    m(1, 0) = 1.0f;
    m(1, 1) = 3.0f;
    m(1, 2) = 2.0f;
    m(2, 0) = 1.0f;
    m(2, 1) = 2.0f;
    m(2, 2) = 5.0f;

    for (auto _ : state)
    {
        auto [Q, R] = m.qr();
        auto eigenvalues = m.eigenvaluesQR(50);
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_QR_Then_Eigenvalues_3x3);

BENCHMARK_MAIN();
