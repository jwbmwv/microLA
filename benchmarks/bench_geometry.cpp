// SPDX-License-Identifier: Apache-2.0
/// @file bench_geometry.cpp
/// @brief Geometry intersection performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <benchmark/benchmark.h>
#include <microla/microla.hpp>
#include <microla/geometry.hpp>

using namespace microla;
using namespace microla::geometry;

// ===== Ray-Plane Intersection =====

static void bm_ray_plane_intersection(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 1.0F, 0.0F));
    Plane<float> plane(Vec<float, 3>(0.0F, 1.0F, 0.0F), -5.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = plane.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_plane_intersection);

static void bm_ray_plane_parallel(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    Plane<float> plane(Vec<float, 3>(0.0F, 1.0F, 0.0F), -5.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = plane.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_plane_parallel);

// ===== Ray-AABB Intersection (Slab Method) =====

static void bm_ray_aabb_hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    AABB<float> box(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = box.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_aabb_hit);

static void bm_ray_aabb_miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0F, 5.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    AABB<float> box(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = box.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_aabb_miss);

// ===== Ray-Sphere Intersection =====

static void bm_ray_sphere_hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 2.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_sphere_hit);

static void bm_ray_sphere_miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0F, 5.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 2.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_sphere_miss);

static void bm_ray_sphere_inside(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(1.0F, 0.0F, 0.0F));
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 5.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_sphere_inside);

// ===== Ray-Triangle Intersection (Möller-Trumbore) =====

static void bm_ray_triangle_hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.5F, 0.5F, -5.0F), Vec<float, 3>(0.0F, 0.0F, 1.0F));
    Triangle<float> tri(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(2.0F, 0.0F, 0.0F),
                        Vec<float, 3>(0.0F, 2.0F, 0.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = intersect(ray, tri);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_triangle_hit);

static void bm_ray_triangle_miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(5.0F, 5.0F, -5.0F), Vec<float, 3>(0.0F, 0.0F, 1.0F));
    Triangle<float> tri(Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(2.0F, 0.0F, 0.0F),
                        Vec<float, 3>(0.0F, 2.0F, 0.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        auto result = intersect(ray, tri);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_ray_triangle_miss);

// ===== Frustum Culling =====

static void bm_frustum_contains_point_inside(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    Vec<float, 3> point(0.0F, 0.0F, 0.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = frustum.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_frustum_contains_point_inside);

static void bm_frustum_contains_sphere(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 1.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = frustum.contains(sphere);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_frustum_contains_sphere);

static void bm_frustum_intersects_aabb(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    AABB<float> box(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = frustum.intersects(box);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_frustum_intersects_aabb);

// ===== Geometry Utility Operations =====

static void bm_aabb_contains_point(benchmark::State& state)
{
    AABB<float> box(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));
    Vec<float, 3> point(0.5F, 0.5F, 0.5F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = box.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_aabb_contains_point);

static void bm_aabb_merge(benchmark::State& state)
{
    AABB<float> box1(Vec<float, 3>(-1.0F, -1.0F, -1.0F), Vec<float, 3>(1.0F, 1.0F, 1.0F));
    AABB<float> box2(Vec<float, 3>(0.5F, 0.5F, 0.5F), Vec<float, 3>(2.0F, 2.0F, 2.0F));

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        AABB<float> result = box1.merge(box2);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(bm_aabb_merge);

static void bm_sphere_contains_point(benchmark::State& state)
{
    Sphere<float> sphere(Vec<float, 3>(0.0F, 0.0F, 0.0F), 5.0F);
    Vec<float, 3> point(3.0F, 4.0F, 0.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = sphere.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_sphere_contains_point);

static void bm_sphere_intersects(benchmark::State& state)
{
    Sphere<float> sphere1(Vec<float, 3>(0.0F, 0.0F, 0.0F), 3.0F);
    Sphere<float> sphere2(Vec<float, 3>(4.0F, 0.0F, 0.0F), 3.0F);

    for (auto _ : state)  // NOLINT(readability-identifier-length)
    {
        bool result = sphere1.intersects(sphere2);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(bm_sphere_intersects);

BENCHMARK_MAIN();
