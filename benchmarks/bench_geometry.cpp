// SPDX-License-Identifier: MIT
/// @file bench_geometry.cpp
/// @brief Geometry intersection performance benchmarks
/// @copyright Copyright (c) 2026 James Baldwin

#include <benchmark/benchmark.h>
#include <microla/microla.hpp>
#include <microla/geometry.hpp>

using namespace microla;
using namespace microla::geometry;

// ===== Ray-Plane Intersection =====

static void BM_RayPlane_Intersection(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(0.0f, 1.0f, 0.0f));
    Plane<float> plane(Vec<float, 3>(0.0f, 1.0f, 0.0f), -5.0f);

    for (auto _ : state)
    {
        auto result = plane.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayPlane_Intersection);

static void BM_RayPlane_Parallel(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    Plane<float> plane(Vec<float, 3>(0.0f, 1.0f, 0.0f), -5.0f);

    for (auto _ : state)
    {
        auto result = plane.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayPlane_Parallel);

// ===== Ray-AABB Intersection (Slab Method) =====

static void BM_RayAABB_Hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    AABB<float> box(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));

    for (auto _ : state)
    {
        auto result = box.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayAABB_Hit);

static void BM_RayAABB_Miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 5.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    AABB<float> box(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));

    for (auto _ : state)
    {
        auto result = box.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayAABB_Miss);

// ===== Ray-Sphere Intersection =====

static void BM_RaySphere_Hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 2.0f);

    for (auto _ : state)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RaySphere_Hit);

static void BM_RaySphere_Miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 5.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 2.0f);

    for (auto _ : state)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RaySphere_Miss);

static void BM_RaySphere_Inside(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 5.0f);

    for (auto _ : state)
    {
        auto result = sphere.intersect(ray);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RaySphere_Inside);

// ===== Ray-Triangle Intersection (Möller-Trumbore) =====

static void BM_RayTriangle_Hit(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(0.5f, 0.5f, -5.0f), Vec<float, 3>(0.0f, 0.0f, 1.0f));
    Triangle<float> tri(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(2.0f, 0.0f, 0.0f),
                        Vec<float, 3>(0.0f, 2.0f, 0.0f));

    for (auto _ : state)
    {
        auto result = intersect(ray, tri);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayTriangle_Hit);

static void BM_RayTriangle_Miss(benchmark::State& state)
{
    Ray<float> ray(Vec<float, 3>(5.0f, 5.0f, -5.0f), Vec<float, 3>(0.0f, 0.0f, 1.0f));
    Triangle<float> tri(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(2.0f, 0.0f, 0.0f),
                        Vec<float, 3>(0.0f, 2.0f, 0.0f));

    for (auto _ : state)
    {
        auto result = intersect(ray, tri);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RayTriangle_Miss);

// ===== Frustum Culling =====

static void BM_Frustum_ContainsPoint_Inside(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    Vec<float, 3> point(0.0f, 0.0f, 0.0f);

    for (auto _ : state)
    {
        bool result = frustum.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Frustum_ContainsPoint_Inside);

static void BM_Frustum_ContainsSphere(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 1.0f);

    for (auto _ : state)
    {
        bool result = frustum.contains(sphere);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Frustum_ContainsSphere);

static void BM_Frustum_IntersectsAABB(benchmark::State& state)
{
    auto vp = SquareMat<float, 4>::identity();
    Frustum<float> frustum = Frustum<float>::from_matrix(vp);
    AABB<float> box(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));

    for (auto _ : state)
    {
        bool result = frustum.intersects(box);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Frustum_IntersectsAABB);

// ===== Geometry Utility Operations =====

static void BM_AABB_ContainsPoint(benchmark::State& state)
{
    AABB<float> box(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));
    Vec<float, 3> point(0.5f, 0.5f, 0.5f);

    for (auto _ : state)
    {
        bool result = box.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AABB_ContainsPoint);

static void BM_AABB_Merge(benchmark::State& state)
{
    AABB<float> box1(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));
    AABB<float> box2(Vec<float, 3>(0.5f, 0.5f, 0.5f), Vec<float, 3>(2.0f, 2.0f, 2.0f));

    for (auto _ : state)
    {
        AABB<float> result = box1.merge(box2);
        benchmark::DoNotOptimize(result);
        benchmark::ClobberMemory();
    }
}
BENCHMARK(BM_AABB_Merge);

static void BM_Sphere_ContainsPoint(benchmark::State& state)
{
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 5.0f);
    Vec<float, 3> point(3.0f, 4.0f, 0.0f);

    for (auto _ : state)
    {
        bool result = sphere.contains(point);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Sphere_ContainsPoint);

static void BM_Sphere_Intersects(benchmark::State& state)
{
    Sphere<float> sphere1(Vec<float, 3>(0.0f, 0.0f, 0.0f), 3.0f);
    Sphere<float> sphere2(Vec<float, 3>(4.0f, 0.0f, 0.0f), 3.0f);

    for (auto _ : state)
    {
        bool result = sphere1.intersects(sphere2);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Sphere_Intersects);

BENCHMARK_MAIN();
