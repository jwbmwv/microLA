// SPDX-License-Identifier: Apache-2.0
/// @file bench_algorithms.cpp
/// @brief Numerical algorithm performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

// ===== QR Decomposition Benchmarks =====

static void bm_qr_3x3(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 12.0F;
    m(0, 1) = -51.0F;
    m(0, 2) = 4.0F;
    m(1, 0) = 6.0F;
    m(1, 1) = 167.0F;
    m(1, 2) = -68.0F;
    m(2, 0) = -4.0F;
    m(2, 1) = 24.0F;
    m(2, 2) = -41.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_qr_3x3);

static void bm_qr_4x4(benchmark::State& state)
{
    SquareMat<float, 4> m;
    for (uint32_t i = 0; i < 4; ++i)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            m(i, j) = static_cast<float>(i * 4 + j + 1);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_qr_4x4);

static void bm_qr_5x5(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            m(i, j) = static_cast<float>(i * 5 + j + 1);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto [Q, R] = m.qr();
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_qr_5x5);

// ===== Eigenvalue Computation Benchmarks =====

static void bm_eigenvalues_3x3_identity(benchmark::State& state)
{
    SquareMat<float, 3> m = SquareMat<float, 3>::identity();

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto eigenvalues = m.eigenvalues_qr(100);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_eigenvalues_3x3_identity);

static void bm_eigenvalues_3x3_symmetric(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 4.0F;
    m(0, 1) = 1.0F;
    m(0, 2) = 1.0F;
    m(1, 0) = 1.0F;
    m(1, 1) = 3.0F;
    m(1, 2) = 2.0F;
    m(2, 0) = 1.0F;
    m(2, 1) = 2.0F;
    m(2, 2) = 5.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto eigenvalues = m.eigenvalues_qr(100);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_eigenvalues_3x3_symmetric);

// ===== Determinant Benchmarks =====

static void bm_determinant_4x4(benchmark::State& state)
{
    SquareMat<float, 4> m;
    m(0, 0) = 1.0F;
    m(0, 1) = 2.0F;
    m(0, 2) = 3.0F;
    m(0, 3) = 4.0F;
    m(1, 0) = 5.0F;
    m(1, 1) = 6.0F;
    m(1, 2) = 7.0F;
    m(1, 3) = 8.0F;
    m(2, 0) = 9.0F;
    m(2, 1) = 2.0F;
    m(2, 2) = 3.0F;
    m(2, 3) = 1.0F;
    m(3, 0) = 4.0F;
    m(3, 1) = 5.0F;
    m(3, 2) = 6.0F;
    m(3, 3) = 7.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(bm_determinant_4x4);

static void bm_determinant_5x5(benchmark::State& state)
{
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            m(i, j) = static_cast<float>((i + j) % 5 + 1);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float det = m.determinant();
        benchmark::DoNotOptimize(det);
    }
}
BENCHMARK(bm_determinant_5x5);

// ===== Matrix Inverse Benchmarks =====

static void bm_inverse_4x4(benchmark::State& state)
{
    // Use a known invertible 4x4 matrix (det ≠ 0)
    SquareMat<float, 4> m;
    m(0, 0) = 4.0F;
    m(0, 1) = 3.0F;
    m(0, 2) = 0.0F;
    m(0, 3) = 0.0F;
    m(1, 0) = 3.0F;
    m(1, 1) = 2.0F;
    m(1, 2) = 0.0F;
    m(1, 3) = 0.0F;
    m(2, 0) = 0.0F;
    m(2, 1) = 0.0F;
    m(2, 2) = 1.0F;
    m(2, 3) = 0.5F;
    m(3, 0) = 0.0F;
    m(3, 1) = 0.0F;
    m(3, 2) = 0.5F;
    m(3, 3) = 1.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 4> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_inverse_4x4);

static void bm_inverse_5x5(benchmark::State& state)
{
    // Use diagonal-dominant matrix (guaranteed invertible)
    SquareMat<float, 5> m;
    for (uint32_t i = 0; i < 5; ++i)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            m(i, j) = ((i == j) ? 10.0F : (i + j) % 3 + 0.5F);
        }
    }

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        SquareMat<float, 5> inv = m.inverse();
        benchmark::DoNotOptimize(inv);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_inverse_5x5);

// ===== Combined Algorithm Workflow =====

static void bm_qr_then_eigenvalues_3x3(benchmark::State& state)
{
    SquareMat<float, 3> m;
    m(0, 0) = 4.0F;
    m(0, 1) = 1.0F;
    m(0, 2) = 1.0F;
    m(1, 0) = 1.0F;
    m(1, 1) = 3.0F;
    m(1, 2) = 2.0F;
    m(2, 0) = 1.0F;
    m(2, 1) = 2.0F;
    m(2, 2) = 5.0F;

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto [Q, R] = m.qr();
        auto eigenvalues = m.eigenvalues_qr(50);
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(R);
        benchmark::DoNotOptimize(eigenvalues);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_qr_then_eigenvalues_3x3);

BENCHMARK_MAIN();
