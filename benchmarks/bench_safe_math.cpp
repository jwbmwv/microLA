// SPDX-License-Identifier: Apache-2.0
// Simple benchmarks for microla::safe functions
#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

static void bm_safe_divide_edge_cases(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        float a = 1.0F;
        float b = (state.iterations() % 2 == 0) ? 0.0F : 1e-8F;
        float r = safe::safe_divide(a, b, -1.0F);
        benchmark::DoNotOptimize(r);
    }
}
BENCHMARK(bm_safe_divide_edge_cases)->Unit(benchmark::kNanosecond);

static void bm_saturating_add_int(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        int a = std::numeric_limits<int>::max() - 1;
        int b = 10;
        int r = safe::saturating_add(a, b);
        benchmark::DoNotOptimize(r);
    }
}
BENCHMARK(bm_saturating_add_int)->Unit(benchmark::kNanosecond);

static void bm_safe_sqrt_positive_negative(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        double v = (state.iterations() % 2 == 0) ? 4.0 : -1.0;
        double r = safe::safe_sqrt(v);
        benchmark::DoNotOptimize(r);
    }
}
BENCHMARK(bm_safe_sqrt_positive_negative)->Unit(benchmark::kNanosecond);

// Saturating multiply for 32-bit unsigned
static void bm_saturating_mul_u32(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        uint32_t a = std::numeric_limits<uint32_t>::max() / 2;
        uint32_t b = 4;
        uint32_t r = safe::saturating_mul(a, b);
        benchmark::DoNotOptimize(r);
    }
}
BENCHMARK(bm_saturating_mul_u32)->Unit(benchmark::kNanosecond);

// Run all
BENCHMARK_MAIN();
