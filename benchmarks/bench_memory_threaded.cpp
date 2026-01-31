// SPDX-License-Identifier: Apache-2.0
// Memory allocation and simple threaded benchmarks
#include <microla/microla.hpp>
#include <benchmark/benchmark.h>
#include <thread>
#include <vector>

using namespace microla;

static void bm_heap_alloc_matrices(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        std::vector<Mat<float, 4, 4>*> v;
        v.reserve(1000);
        for (int i = 0; i < 1000; ++i)
        {
            v.push_back(new Mat<float, 4, 4>(Mat<float, 4, 4>::identity()));
        }

        float sum = 0.0F;
        for (auto* p : v)
        {
            sum += p->operator()(0, 0);
        }
        for (auto* p : v)
        {
            delete p;
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_heap_alloc_matrices)->Unit(benchmark::kMillisecond)->Iterations(20);

static void bm_stack_alloc_matrices(benchmark::State& state)
{
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        Mat<float, 4, 4> v[1000];
        for (auto& i : v)
        {
            i = Mat<float, 4, 4>::identity();
        }
        float sum = 0.0F;
        for (auto& i : v)
        {
            sum += i(0, 0);
        }
        benchmark::DoNotOptimize(sum);
    }
}
BENCHMARK(bm_stack_alloc_matrices)->Unit(benchmark::kMillisecond)->Iterations(20);

// Threaded: run matrix multiplies concurrently
static void bm_threaded_mat_multiply(benchmark::State& state)
{
    const int threads = 4;
    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        std::vector<std::thread> workers;
        workers.reserve(threads);
        for (int t = 0; t < threads; ++t)
        {
            workers.emplace_back(
                []
                {
                    Mat<float, 8, 8> a = Mat<float, 8, 8>::identity();
                    Mat<float, 8, 8> b = Mat<float, 8, 8>::identity();
                    for (int i = 0; i < 1000; ++i)
                    {
                        Mat<float, 8, 8> c = a * b;
                        benchmark::DoNotOptimize(c);
                    }
                });
        }
        for (auto& w : workers)
        {
            w.join();
        }
    }
}
BENCHMARK(bm_threaded_mat_multiply)->Unit(benchmark::kMillisecond)->Iterations(5);

BENCHMARK_MAIN();
