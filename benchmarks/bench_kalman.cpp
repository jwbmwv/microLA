// SPDX-License-Identifier: Apache-2.0
// Small Kalman filter pipeline benchmark
#include <microla/microla.hpp>
#include <benchmark/benchmark.h>

using namespace microla;

static void bm_kalman_4_d_2_meas(benchmark::State& state)
{
    using KF = KalmanFilter<float, 4, 2>;
    KF kf;

    // Simple constant motion model
    Mat<float, 4, 4> f = Mat<float, 4, 4>::identity();
    kf.set_state_transition(f);

    // measurement matrix: measure position components
    Mat<float, 2, 4> h{};
    h(0, 0) = 1.0F;
    h(1, 2) = 1.0F;
    kf.set_measurement_matrix(h);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        kf.predict();
        Vec<float, 2> z;
        z[0] = 1.0F;
        z[1] = 0.5F;
        kf.update(z);
        benchmark::DoNotOptimize(kf.get_state());
    }
}
BENCHMARK(bm_kalman_4_d_2_meas)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
