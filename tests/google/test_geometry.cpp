// SPDX-License-Identifier: MIT
/// @file test_geometry.cpp
/// @brief Comprehensive tests for geometry classes
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <gtest/gtest.h>

#if __cplusplus >= 201703L
#include <microla/geometry.hpp>
#include <cmath>

using namespace microla;
using namespace microla::geometry;

// ===== Ray Tests =====

class RayTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        origin = Vec<float, 3>(0.0f, 0.0f, 0.0f);
        direction = Vec<float, 3>(1.0f, 0.0f, 0.0f);
        ray = Ray<float>(origin, direction);
    }

    Vec<float, 3> origin, direction;
    Ray<float> ray;
    const float epsilon = 1e-6f;
};

TEST_F(RayTest, Construction)
{
    EXPECT_EQ(ray.origin, origin);
    EXPECT_EQ(ray.direction, direction);
}

TEST_F(RayTest, At)
{
    Vec<float, 3> point = ray.at(5.0f);
    EXPECT_FLOAT_EQ(point[0], 5.0f);
    EXPECT_FLOAT_EQ(point[1], 0.0f);
    EXPECT_FLOAT_EQ(point[2], 0.0f);
}

TEST_F(RayTest, AtNegative)
{
    Vec<float, 3> point = ray.at(-2.0f);
    EXPECT_FLOAT_EQ(point[0], -2.0f);
}

// ===== Plane Tests =====

class PlaneTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        normal = Vec<float, 3>(0.0f, 1.0f, 0.0f);
        d = -5.0f;
        plane = Plane<float>(normal, d);

        point_on_plane = Vec<float, 3>(0.0f, 5.0f, 0.0f);
        up_normal = Vec<float, 3>(0.0f, 1.0f, 0.0f);
        plane_from_point = Plane<float>::from_point_normal(point_on_plane, up_normal);
    }

    Vec<float, 3> normal, point_on_plane, up_normal;
    float d;
    Plane<float> plane, plane_from_point;
    const float epsilon = 1e-6f;
};

TEST_F(PlaneTest, Construction)
{
    EXPECT_EQ(plane.normal, normal);
    EXPECT_FLOAT_EQ(plane.d, d);
}

TEST_F(PlaneTest, FromPointNormal)
{
    float dist = plane_from_point.distance(point_on_plane);
    EXPECT_NEAR(dist, 0.0f, epsilon);
}

TEST_F(PlaneTest, Distance)
{
    Vec<float, 3> point(0.0f, 10.0f, 0.0f);
    float dist = plane.distance(point);
    EXPECT_FLOAT_EQ(dist, 5.0f);
}

TEST_F(PlaneTest, SignedDistance)
{
    Vec<float, 3> above(0.0f, 10.0f, 0.0f);
    Vec<float, 3> below(0.0f, 0.0f, 0.0f);

    EXPECT_GT(plane.signed_distance(above), 0.0f);
    EXPECT_LT(plane.signed_distance(below), 0.0f);
}

TEST_F(PlaneTest, Project)
{
    Vec<float, 3> point(5.0f, 10.0f, 3.0f);
    Vec<float, 3> projected = plane.project(point);

    EXPECT_FLOAT_EQ(projected[0], 5.0f);
    EXPECT_NEAR(projected[1], 5.0f, epsilon);
    EXPECT_FLOAT_EQ(projected[2], 3.0f);
}

TEST_F(PlaneTest, RayIntersection)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(0.0f, 1.0f, 0.0f));
    auto result = plane.intersect(ray);

    EXPECT_TRUE(result.has_value());
    if (result)
    {
        EXPECT_NEAR(result.value(), 5.0f, epsilon);
    }
}

TEST_F(PlaneTest, ParallelRay)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    auto result = plane.intersect(ray);

    EXPECT_FALSE(result.has_value());
}

// ===== AABB Tests =====

class AABBTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        min = Vec<float, 3>(-1.0f, -1.0f, -1.0f);
        max = Vec<float, 3>(1.0f, 1.0f, 1.0f);
        box = AABB<float>(min, max);

        center = Vec<float, 3>(0.0f, 0.0f, 0.0f);
        extents = Vec<float, 3>(1.0f, 1.0f, 1.0f);
        box_from_center = AABB<float>::from_center_extents(center, extents);
    }

    Vec<float, 3> min, max, center, extents;
    AABB<float> box, box_from_center;
    const float epsilon = 1e-6f;
};

TEST_F(AABBTest, Construction)
{
    EXPECT_EQ(box.min, min);
    EXPECT_EQ(box.max, max);
}

TEST_F(AABBTest, FromCenterExtents)
{
    EXPECT_NEAR(box_from_center.min[0], -1.0f, epsilon);
    EXPECT_NEAR(box_from_center.max[0], 1.0f, epsilon);
}

TEST_F(AABBTest, Center)
{
    Vec<float, 3> c = box.center();
    EXPECT_NEAR(c[0], 0.0f, epsilon);
    EXPECT_NEAR(c[1], 0.0f, epsilon);
    EXPECT_NEAR(c[2], 0.0f, epsilon);
}

TEST_F(AABBTest, Extents)
{
    Vec<float, 3> e = box.extents();
    EXPECT_NEAR(e[0], 1.0f, epsilon);
    EXPECT_NEAR(e[1], 1.0f, epsilon);
    EXPECT_NEAR(e[2], 1.0f, epsilon);
}

TEST_F(AABBTest, Size)
{
    Vec<float, 3> s = box.size();
    EXPECT_NEAR(s[0], 2.0f, epsilon);
    EXPECT_NEAR(s[1], 2.0f, epsilon);
    EXPECT_NEAR(s[2], 2.0f, epsilon);
}

TEST_F(AABBTest, ContainsPoint)
{
    Vec<float, 3> inside(0.0f, 0.0f, 0.0f);
    Vec<float, 3> outside(2.0f, 0.0f, 0.0f);
    Vec<float, 3> on_edge(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(box.contains(inside));
    EXPECT_FALSE(box.contains(outside));
    EXPECT_TRUE(box.contains(on_edge));
}

TEST_F(AABBTest, ContainsAABB)
{
    AABB<float> smaller(Vec<float, 3>(-0.5f, -0.5f, -0.5f), Vec<float, 3>(0.5f, 0.5f, 0.5f));
    AABB<float> larger(Vec<float, 3>(-2.0f, -2.0f, -2.0f), Vec<float, 3>(2.0f, 2.0f, 2.0f));

    EXPECT_TRUE(box.contains(smaller));
    EXPECT_FALSE(box.contains(larger));
}

TEST_F(AABBTest, Intersects)
{
    AABB<float> overlapping(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(2.0f, 2.0f, 2.0f));
    AABB<float> separate(Vec<float, 3>(2.0f, 2.0f, 2.0f), Vec<float, 3>(3.0f, 3.0f, 3.0f));

    EXPECT_TRUE(box.intersects(overlapping));
    EXPECT_FALSE(box.intersects(separate));
}

TEST_F(AABBTest, RayIntersection)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    auto result = box.intersect(ray);

    EXPECT_TRUE(result.has_value());
    if (result)
    {
        auto [t_min, t_max] = result.value();
        EXPECT_NEAR(t_min, 4.0f, epsilon);
        EXPECT_NEAR(t_max, 6.0f, epsilon);
    }
}

TEST_F(AABBTest, Merge)
{
    AABB<float> other(Vec<float, 3>(0.5f, 0.5f, 0.5f), Vec<float, 3>(2.0f, 2.0f, 2.0f));
    AABB<float> merged = box.merge(other);

    EXPECT_NEAR(merged.min[0], -1.0f, epsilon);
    EXPECT_NEAR(merged.max[0], 2.0f, epsilon);
}

TEST_F(AABBTest, Expand)
{
    AABB<float> expanded = box.expand(0.5f);

    EXPECT_NEAR(expanded.min[0], -1.5f, epsilon);
    EXPECT_NEAR(expanded.max[0], 1.5f, epsilon);
}

// ===== Sphere Tests =====

class SphereTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        center = Vec<float, 3>(0.0f, 0.0f, 0.0f);
        radius = 1.0f;
        sphere = Sphere<float>(center, radius);
    }

    Vec<float, 3> center;
    float radius;
    Sphere<float> sphere;
    const float epsilon = 1e-6f;
};

TEST_F(SphereTest, Construction)
{
    EXPECT_EQ(sphere.center, center);
    EXPECT_FLOAT_EQ(sphere.radius, radius);
}

TEST_F(SphereTest, ContainsPoint)
{
    Vec<float, 3> inside(0.5f, 0.0f, 0.0f);
    Vec<float, 3> outside(2.0f, 0.0f, 0.0f);
    Vec<float, 3> on_surface(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(sphere.contains(inside));
    EXPECT_FALSE(sphere.contains(outside));
    EXPECT_TRUE(sphere.contains(on_surface));
}

TEST_F(SphereTest, RayIntersection)
{
    Ray<float> ray(Vec<float, 3>(-5.0f, 0.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    auto result = sphere.intersect(ray);

    EXPECT_TRUE(result.has_value());
    if (result)
    {
        auto [t1, t2] = result.value();
        EXPECT_NEAR(t1, 4.0f, epsilon);
        EXPECT_NEAR(t2, 6.0f, epsilon);
    }
}

TEST_F(SphereTest, RayMiss)
{
    Ray<float> ray(Vec<float, 3>(0.0f, 5.0f, 0.0f), Vec<float, 3>(1.0f, 0.0f, 0.0f));
    auto result = sphere.intersect(ray);

    EXPECT_FALSE(result.has_value());
}

TEST_F(SphereTest, ContainsSphere)
{
    Sphere<float> smaller(center, 0.5f);
    Sphere<float> larger(center, 2.0f);

    EXPECT_TRUE(sphere.contains(smaller));
    EXPECT_FALSE(sphere.contains(larger));
}

TEST_F(SphereTest, Intersects)
{
    Sphere<float> overlapping(Vec<float, 3>(1.5f, 0.0f, 0.0f), 1.0f);
    Sphere<float> separate(Vec<float, 3>(3.0f, 0.0f, 0.0f), 1.0f);

    EXPECT_TRUE(sphere.intersects(overlapping));
    EXPECT_FALSE(sphere.intersects(separate));
}

// ===== Triangle Tests =====

class TriangleTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        v0 = Vec<float, 3>(0.0f, 0.0f, 0.0f);
        v1 = Vec<float, 3>(1.0f, 0.0f, 0.0f);
        v2 = Vec<float, 3>(0.0f, 1.0f, 0.0f);
        triangle = Triangle<float>(v0, v1, v2);
    }

    Vec<float, 3> v0, v1, v2;
    Triangle<float> triangle;
    const float epsilon = 1e-6f;
};

TEST_F(TriangleTest, Construction)
{
    EXPECT_EQ(triangle.v0, v0);
    EXPECT_EQ(triangle.v1, v1);
    EXPECT_EQ(triangle.v2, v2);
}

TEST_F(TriangleTest, Normal)
{
    Vec<float, 3> n = triangle.normal();
    EXPECT_NEAR(n[0], 0.0f, epsilon);
    EXPECT_NEAR(n[1], 0.0f, epsilon);
    EXPECT_NEAR(n[2], 1.0f, epsilon);
}

TEST_F(TriangleTest, Area)
{
    float area = triangle.area();
    EXPECT_NEAR(area, 0.5f, epsilon);
}

TEST_F(TriangleTest, Centroid)
{
    Vec<float, 3> c = triangle.centroid();
    EXPECT_NEAR(c[0], 1.0f / 3.0f, epsilon);
    EXPECT_NEAR(c[1], 1.0f / 3.0f, epsilon);
    EXPECT_NEAR(c[2], 0.0f, epsilon);
}

TEST_F(TriangleTest, ContainsPoint)
{
    Vec<float, 3> inside(0.25f, 0.25f, 0.0f);
    Vec<float, 3> outside(2.0f, 2.0f, 0.0f);

    EXPECT_TRUE(triangle.contains(inside));
    EXPECT_FALSE(triangle.contains(outside));
}

TEST_F(TriangleTest, RayIntersection)
{
    Ray<float> ray(Vec<float, 3>(0.25f, 0.25f, -1.0f), Vec<float, 3>(0.0f, 0.0f, 1.0f));
    auto result = intersect(ray, triangle);

    EXPECT_TRUE(result.has_value());
    if (result)
    {
        auto [t, u, v] = result.value();
        EXPECT_NEAR(t, 1.0f, epsilon);
    }
}

TEST_F(TriangleTest, RayMiss)
{
    Ray<float> ray(Vec<float, 3>(2.0f, 2.0f, -1.0f), Vec<float, 3>(0.0f, 0.0f, 1.0f));
    auto result = intersect(ray, triangle);

    EXPECT_FALSE(result.has_value());
}

// ===== Frustum Tests =====

class FrustumTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create a simple frustum from a view-projection matrix
        Mat<float, 4, 4> vp = Mat<float, 4, 4>::identity();
        frustum = Frustum<float>::from_matrix(vp);
    }

    Frustum<float> frustum;
    const float epsilon = 1e-6f;
};

TEST_F(FrustumTest, Construction)
{
    EXPECT_EQ(frustum.planes.size(), 6);
}

TEST_F(FrustumTest, ContainsPoint)
{
    Vec<float, 3> origin(0.0f, 0.0f, 0.0f);
    // Point at origin should be inside a standard frustum
    EXPECT_TRUE(frustum.contains(origin));
}

TEST_F(FrustumTest, ContainsSphere)
{
    Sphere<float> small_sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 0.1f);
    EXPECT_TRUE(frustum.contains(small_sphere));
}

TEST_F(FrustumTest, IntersectsAABB)
{
    AABB<float> small_box(Vec<float, 3>(-0.1f, -0.1f, -0.1f), Vec<float, 3>(0.1f, 0.1f, 0.1f));
    EXPECT_TRUE(frustum.intersects(small_box));
}

// ===== Intersection Tests =====

TEST(GeometryIntersectionTest, SphereSphereIntersection)
{
    Sphere<float> s1(Vec<float, 3>(0.0f, 0.0f, 0.0f), 1.0f);
    Sphere<float> s2(Vec<float, 3>(1.5f, 0.0f, 0.0f), 1.0f);
    Sphere<float> s3(Vec<float, 3>(3.0f, 0.0f, 0.0f), 1.0f);

    EXPECT_TRUE(s1.intersects(s2));
    EXPECT_FALSE(s1.intersects(s3));
}

TEST(GeometryIntersectionTest, AABBAABBIntersection)
{
    AABB<float> box1(Vec<float, 3>(-1.0f, -1.0f, -1.0f), Vec<float, 3>(1.0f, 1.0f, 1.0f));
    AABB<float> box2(Vec<float, 3>(0.0f, 0.0f, 0.0f), Vec<float, 3>(2.0f, 2.0f, 2.0f));
    AABB<float> box3(Vec<float, 3>(2.0f, 2.0f, 2.0f), Vec<float, 3>(3.0f, 3.0f, 3.0f));

    EXPECT_TRUE(box1.intersects(box2));
    EXPECT_FALSE(box1.intersects(box3));
}

TEST(GeometryIntersectionTest, SphereAABBIntersection)
{
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 1.0f);
    AABB<float> box1(Vec<float, 3>(-0.5f, -0.5f, -0.5f), Vec<float, 3>(0.5f, 0.5f, 0.5f));
    AABB<float> box2(Vec<float, 3>(2.0f, 2.0f, 2.0f), Vec<float, 3>(3.0f, 3.0f, 3.0f));

    EXPECT_TRUE(sphere.intersects(box1));
    EXPECT_FALSE(sphere.intersects(box2));
}

// ===== Edge Cases =====

TEST(GeometryEdgeCaseTest, DegenerateTriangle)
{
    Vec<float, 3> v0(0.0f, 0.0f, 0.0f);
    Vec<float, 3> v1(0.0f, 0.0f, 0.0f);
    Vec<float, 3> v2(0.0f, 0.0f, 0.0f);
    Triangle<float> triangle(v0, v1, v2);

    float area = triangle.area();
    EXPECT_FLOAT_EQ(area, 0.0f);
}

TEST(GeometryEdgeCaseTest, ZeroRadiusSphere)
{
    Sphere<float> sphere(Vec<float, 3>(0.0f, 0.0f, 0.0f), 0.0f);
    Vec<float, 3> point(0.0f, 0.0f, 0.0f);

    EXPECT_TRUE(sphere.contains(point));
}

TEST(GeometryEdgeCaseTest, InvertedAABB)
{
    AABB<float> box(Vec<float, 3>(1.0f, 1.0f, 1.0f), Vec<float, 3>(-1.0f, -1.0f, -1.0f));
    // Should handle gracefully
    Vec<float, 3> size = box.size();
    EXPECT_TRUE(std::isfinite(size[0]));
}

#else
// Geometry tests require C++17 for std::optional
TEST(GeometryTest, RequiresCpp17)
{
    GTEST_SKIP() << "Geometry module requires C++17 or later";
}
#endif
