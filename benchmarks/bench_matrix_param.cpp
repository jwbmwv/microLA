// SPDX-License-Identifier: Apache-2.0
// Parameterized matrix-size and type benchmarks
#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

#define REGISTER_MAT_BENCH(TYPE, N)                                      \
    static void BM_MatMultiply_##TYPE##_##N(benchmark::State& state)     \
    {                                                                    \
        using M = Mat<TYPE, N, N>;                                       \
        M A = M::identity();                                             \
        M B = M::identity();                                             \
        for (auto _ : state) /* NOLINT(readability-identifier-length) */ \
        {                                                                \
            M C = A * B;                                                 \
            benchmark::DoNotOptimize(C);                                 \
        }                                                                \
    }                                                                    \
    BENCHMARK(BM_MatMultiply_##TYPE##_##N)->Unit(benchmark::kNanosecond);

// Instantiate a few sizes and types
REGISTER_MAT_BENCH(float, 4)
REGISTER_MAT_BENCH(float, 8)
REGISTER_MAT_BENCH(float, 16)

REGISTER_MAT_BENCH(double, 4)
REGISTER_MAT_BENCH(double, 8)
REGISTER_MAT_BENCH(double, 16)

// MatrixView microbenchmark: small block copy/read
static void bm_matrix_view_block_access(benchmark::State& state)
{
    using M4 = Mat<float, 8, 8>;
    M4 full = M4::identity();
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        // Top-left 4x4 view
        MatrixView<float, 4, 4> v(full.data_ptr(), 8, 0, 0);
        float sum = 0.0F;
        for (uint32_t r = 0; r < 4; ++r)
        {
            for (uint32_t c = 0; c < 4; ++c)
            {
                sum += v(r, c);
            }
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_matrix_view_block_access)->Unit(benchmark::kNanosecond);

BENCHMARK_MAIN();
