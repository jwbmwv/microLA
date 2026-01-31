// SPDX-License-Identifier: Apache-2.0
/// @file geometry.hpp
/// @brief Geometric primitives and utilities for collision detection and ray tracing
/// @details Provides AABB, spheres, rays, planes, and intersection tests
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "vector.hpp"
#include "matrix.hpp"
#include <optional>
#include <array>
#include <algorithm>
#include <cmath>
#include "high_precision.hpp"

namespace microla
{
namespace geometry
{

// ==================== Ray ====================

/// @brief Ray defined by origin and direction.
/// @tparam T Scalar type for coordinates.
template<typename T = float>
struct Ray
{
    /// @brief Ray origin point.
    Vec<T, 3> origin;

    /// @brief Ray direction (should be normalized for accurate distances).
    Vec<T, 3> direction;

    /// @brief Default constructor.
    Ray() : origin(), direction(Vec<T, 3>(T(0), T(0), T(1))) {}

    /// @brief Construct a ray from origin and direction.
    /// @param o Origin point.
    /// @param d Direction vector (prefer normalized).
    Ray(const Vec<T, 3>& o, const Vec<T, 3>& d) : origin(o), direction(d) {}

    /// @brief Get point along ray at parameter t.
    /// @param t Distance along ray.
    /// @return Point at origin + direction * t.
    [[nodiscard]] auto at(T t) const -> Vec<T, 3> { return origin + direction * t; }
};

using Rayf = Ray<float>;
using Rayd = Ray<double>;

// ==================== Plane ====================

/// @brief Plane defined by normal and signed distance from origin.
/// @tparam T Scalar type for coordinates.
template<typename T = float>
struct Plane
{
    /// @brief Unit normal vector.
    Vec<T, 3> normal;

    /// @brief Signed distance from origin along the normal.
    T d;

    /// @brief Default constructor.
    Plane() : normal(Vec<T, 3>(T(0), T(1), T(0))), d(T(0)) {}

    /// @brief Construct plane from normal and distance.
    /// @param n Normal vector (will be normalized).
    /// @param dist Signed distance from origin.
    Plane(const Vec<T, 3>& n, T dist) : normal(n.normalized()), d(dist) {}

    /// @brief Construct plane from normal and a point on the plane.
    /// @param point Any point on the plane.
    /// @param normal Plane normal (will be normalized).
    /// @return Plane instance.
    [[nodiscard]] static auto from_point_normal(const Vec<T, 3>& point, const Vec<T, 3>& normal) -> Plane
    {
        Vec<T, 3> n = normal.normalized();
        return Plane(n, -n.dot(point));
    }

    /// @brief Construct plane from three points (counter-clockwise).
    /// @param p0 First point on plane.
    /// @param p1 Second point on plane.
    /// @param p2 Third point on plane.
    /// @return Plane instance.
    [[nodiscard]] static auto from_points(const Vec<T, 3>& p0, const Vec<T, 3>& p1, const Vec<T, 3>& p2) -> Plane
    {
        Vec<T, 3> v1 = p1 - p0;
        Vec<T, 3> v2 = p2 - p0;
        Vec<T, 3> n = v1.cross(v2).normalized();
        return Plane(n, -n.dot(p0));
    }

    /// @brief Signed distance from point to plane (positive = in front).
    /// @param point Point to measure.
    /// @return Signed distance.
    [[nodiscard]] auto distance(const Vec<T, 3>& point) const -> T { return normal.dot(point) + d; }

    /// @brief Signed distance from point to plane.
    [[nodiscard]] auto signed_distance(const Vec<T, 3>& point) const -> T { return distance(point); }

    /// @brief Project a point onto the plane.
    /// @param point Point to project.
    /// @return Projected point on the plane.
    [[nodiscard]] auto project(const Vec<T, 3>& point) const -> Vec<T, 3>
    {
        // Compute projection components using a selectable higher-precision
        // intermediate type to avoid unnecessary long-double bloat on embedded
        // targets.
        using high_t = microla::detail::hi_t<T>;

        auto nx = static_cast<high_t>(normal.x());
        auto ny = static_cast<high_t>(normal.y());
        auto nz = static_cast<high_t>(normal.z());

        auto px = static_cast<high_t>(point.x());
        auto py = static_cast<high_t>(point.y());
        auto pz = static_cast<high_t>(point.z());

        auto dd = static_cast<high_t>(d);

        high_t dist = nx * px + ny * py + nz * pz + dd;

        T rx = static_cast<T>(px - nx * dist);
        T ry = static_cast<T>(py - ny * dist);
        T rz = static_cast<T>(pz - nz * dist);

        return Vec<T, 3>(rx, ry, rz);
    }

    /// @brief Ray-plane intersection.
    /// @param ray Ray to test.
    /// @return Distance along ray to intersection point, or std::nullopt if no intersection.
    [[nodiscard]] auto intersect(const Ray<T>& ray) const -> std::optional<T>
    {
        T denom = normal.dot(ray.direction);

        // Check if ray is parallel to plane
        if (std::abs(denom) < std::numeric_limits<T>::epsilon())
        {
            return std::nullopt;
        }

        // For Hessian form n·x + d = 0, solve for t where n·(o + t*dir) + d = 0
        T t = -(normal.dot(ray.origin) + d) / denom;

        return (t >= static_cast<T>(0)) ? std::optional<T>(t) : std::nullopt;
    }
};

using Planef = Plane<float>;
using Planed = Plane<double>;

// ==================== Axis-Aligned Bounding Box ====================

/// @brief Axis-aligned bounding box (AABB).
/// @tparam T Scalar type for coordinates.
template<typename T = float>
struct AABB
{
    /// @brief Minimum corner.
    Vec<T, 3> min;

    /// @brief Maximum corner.
    Vec<T, 3> max;

    /// @brief Default constructor (zero-initialized box).
    AABB() : min(Vec<T, 3>()), max(Vec<T, 3>()) {}

    /// @brief Construct from min and max corners.
    /// @param minimum Minimum corner.
    /// @param maximum Maximum corner.
    AABB(const Vec<T, 3>& minimum, const Vec<T, 3>& maximum) : min(minimum), max(maximum) {}

    /// @brief Construct from center and half-extents.
    /// @param center Center point.
    /// @param extents Half-extents along each axis.
    /// @return AABB instance.
    [[nodiscard]] static auto from_center_extents(const Vec<T, 3>& center, const Vec<T, 3>& extents) -> AABB
    {
        return AABB(center - extents, center + extents);
    }

    /// @brief Get center point.
    /// @return Center of the AABB.
    [[nodiscard]] auto center() const -> Vec<T, 3> { return (min + max) * static_cast<T>(0.5); }

    /// @brief Get half-extents (distance from center to faces).
    /// @return Half-extents along each axis.
    [[nodiscard]] auto extents() const -> Vec<T, 3> { return (max - min) * static_cast<T>(0.5); }

    /// @brief Get size (full width, height, depth).
    /// @return Size along each axis.
    [[nodiscard]] auto size() const -> Vec<T, 3> { return max - min; }

    /// @brief Check if a point is inside the AABB.
    /// @param point Point to test.
    /// @return True if point is inside or on the boundary.
    [[nodiscard]] auto contains(const Vec<T, 3>& point) const -> bool
    {
        return (point.x() >= min.x() && point.x() <= max.x()) && (point.y() >= min.y() && point.y() <= max.y()) &&
               (point.z() >= min.z() && point.z() <= max.z());
    }

    /// @brief Check if two AABBs intersect.
    /// @param other Other AABB.
    /// @return True if they overlap.
    [[nodiscard]] auto intersects(const AABB& other) const -> bool
    {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    /// @brief Merge two AABBs.
    /// @param other Other AABB.
    /// @return AABB that contains both.
    [[nodiscard]] auto merge(const AABB& other) const -> AABB
    {
        return AABB(Vec<T, 3>(std::min(min.x(), other.min.x()), std::min(min.y(), other.min.y()),
                              std::min(min.z(), other.min.z())),
                    Vec<T, 3>(std::max(max.x(), other.max.x()), std::max(max.y(), other.max.y()),
                              std::max(max.z(), other.max.z())));
    }

    /// @brief Expand AABB to include a point.
    /// @param point Point to include.
    void expand(const Vec<T, 3>& point)
    {
        min = Vec<T, 3>(std::min(min.x(), point.x()), std::min(min.y(), point.y()), std::min(min.z(), point.z()));
        max = Vec<T, 3>(std::max(max.x(), point.x()), std::max(max.y(), point.y()), std::max(max.z(), point.z()));
    }

    /// @brief Expand AABB by a uniform amount.
    /// @param amount Amount to expand in all directions.
    /// @return Expanded AABB.
    [[nodiscard]] auto expand(T amount) const -> AABB
    {
        Vec<T, 3> offset(amount, amount, amount);
        return AABB(min - offset, max + offset);
    }

    /// @brief Check if this AABB fully contains another AABB.
    /// @param other Other AABB to test.
    /// @return True if other is fully inside this AABB.
    [[nodiscard]] auto contains(const AABB& other) const -> bool
    {
        return (other.min.x() >= min.x() && other.max.x() <= max.x()) &&
               (other.min.y() >= min.y() && other.max.y() <= max.y()) &&
               (other.min.z() >= min.z() && other.max.z() <= max.z());
    }

    /// @brief Ray-AABB intersection.
    /// @param ray Ray to test.
    /// @return Pair of (t_min, t_max) distances along ray, or std::nullopt if no intersection.
    [[nodiscard]] auto intersect(const Ray<T>& ray) const -> std::optional<std::pair<T, T>>
    {
        T t_min = static_cast<T>(0);
        T t_max = std::numeric_limits<T>::infinity();

        for (int i = 0; i < 3; ++i)
        {
            T inv_d = static_cast<T>(1) / ray.direction[i];
            T t0 = (min[i] - ray.origin[i]) * inv_d;
            T t1 = (max[i] - ray.origin[i]) * inv_d;

            if (inv_d < static_cast<T>(0))
            {
                std::swap(t0, t1);
            }

            t_min = std::max(t_min, t0);
            t_max = std::min(t_max, t1);

            if (t_max < t_min)
            {
                return std::nullopt;
            }
        }

        return (t_min >= static_cast<T>(0)) ? std::optional<std::pair<T, T>>(std::make_pair(t_min, t_max))
                                            : std::nullopt;
    }

    /// @brief Get surface area (useful for BVH cost heuristics).
    /// @return Surface area.
    [[nodiscard]] auto surface_area() const -> T
    {
        Vec<T, 3> s = size();
        return static_cast<T>(2) * (s.x() * s.y() + s.y() * s.z() + s.z() * s.x());
    }

    /// @brief Get volume.
    /// @return Volume.
    [[nodiscard]] auto volume() const -> T
    {
        Vec<T, 3> s = size();
        return s.x() * s.y() * s.z();
    }
};

using AABBf = AABB<float>;
using AABBd = AABB<double>;

// ==================== Sphere ====================

/// @brief Sphere defined by center and radius.
/// @tparam T Scalar type for coordinates.
template<typename T = float>
struct Sphere
{
    /// @brief Sphere center.
    Vec<T, 3> center;

    /// @brief Sphere radius.
    T radius;

    /// @brief Default constructor.
    Sphere() : center(), radius(T(1)) {}

    /// @brief Construct sphere from center and radius.
    /// @param c Center point.
    /// @param r Radius.
    Sphere(const Vec<T, 3>& c, T r) : center(c), radius(r) {}

    /// @brief Check if point is inside sphere.
    /// @param point Point to test.
    /// @return True if point is inside or on the boundary.
    [[nodiscard]] auto contains(const Vec<T, 3>& point) const -> bool
    {
        Vec<T, 3> diff = point - center;
        return diff.dot(diff) <= radius * radius;
    }

    /// @brief Check if two spheres intersect.
    /// @param other Other sphere.
    /// @return True if they intersect.
    [[nodiscard]] auto intersects(const Sphere& other) const -> bool
    {
        Vec<T, 3> diff = center - other.center;
        T dist_sq = diff.dot(diff);
        T radius_sum = radius + other.radius;
        return dist_sq <= radius_sum * radius_sum;
    }

    /// @brief Get AABB containing this sphere.
    /// @return Bounding AABB.
    [[nodiscard]] auto to_aabb() const -> AABB<T>
    {
        Vec<T, 3> extents(radius, radius, radius);
        return AABB<T>(center - extents, center + extents);
    }

    /// @brief Get surface area.
    /// @return Surface area.
    [[nodiscard]] auto surface_area() const -> T { return static_cast<T>(4) * constants::pi<T>() * radius * radius; }

    /// @brief Get volume.
    /// @return Volume.
    [[nodiscard]] auto volume() const -> T
    {
        return static_cast<T>(4.0 / 3.0) * constants::pi<T>() * radius * radius * radius;
    }

    /// @brief Ray-sphere intersection.
    /// @param ray Ray to test.
    /// @return Pair of (t1, t2) distances along ray, or std::nullopt if no intersection.
    [[nodiscard]] auto intersect(const Ray<T>& ray) const -> std::optional<std::pair<T, T>>
    {
        Vec<T, 3> oc = ray.origin - center;

        T a = ray.direction.dot(ray.direction);
        T b = static_cast<T>(2) * oc.dot(ray.direction);
        T c = oc.dot(oc) - radius * radius;

        T discriminant = b * b - static_cast<T>(4) * a * c;

        if (discriminant < static_cast<T>(0))
        {
            return std::nullopt;
        }

        T sqrt_disc = std::sqrt(discriminant);
        T t1 = (-b - sqrt_disc) / (static_cast<T>(2) * a);
        T t2 = (-b + sqrt_disc) / (static_cast<T>(2) * a);

        return std::make_pair(t1, t2);
    }

    /// @brief Check if this sphere fully contains another sphere.
    /// @param other Other sphere to test.
    /// @return True if other is fully inside this sphere.
    [[nodiscard]] auto contains(const Sphere& other) const -> bool
    {
        Vec<T, 3> diff = center - other.center;
        T dist = std::sqrt(diff.dot(diff));
        return dist + other.radius <= radius;
    }

    /// @brief Check if sphere intersects AABB.
    /// @param aabb AABB to test.
    /// @return True if they intersect.
    [[nodiscard]] auto intersects(const AABB<T>& aabb) const -> bool
    {
        // Find closest point on AABB to sphere center
        Vec<T, 3> closest(microla::clamp(center.x(), aabb.min.x(), aabb.max.x()),
                          microla::clamp(center.y(), aabb.min.y(), aabb.max.y()),
                          microla::clamp(center.z(), aabb.min.z(), aabb.max.z()));

        Vec<T, 3> diff = closest - center;
        return diff.dot(diff) <= radius * radius;
    }
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

// ==================== Intersection Tests ====================

/// @brief Ray-sphere intersection.
/// @tparam T Scalar type for coordinates.
/// @param ray Ray to test.
/// @param sphere Sphere to test.
/// @return Distance along ray to intersection point, or std::nullopt if no intersection.
template<typename T>
auto intersect(const Ray<T>& ray, const Sphere<T>& sphere) -> std::optional<T>
{
    Vec<T, 3> oc = ray.origin - sphere.center;

    T a = ray.direction.dot(ray.direction);
    T b = static_cast<T>(2) * oc.dot(ray.direction);
    T c = oc.dot(oc) - sphere.radius * sphere.radius;

    T discriminant = b * b - static_cast<T>(4) * a * c;

    if (discriminant < static_cast<T>(0))
    {
        return std::nullopt;  // No intersection
    }

    T sqrt_disc = std::sqrt(discriminant);
    T t = (-b - sqrt_disc) / (static_cast<T>(2) * a);

    if (t < static_cast<T>(0))
    {
        t = (-b + sqrt_disc) / (static_cast<T>(2) * a);
    }

    return (t >= static_cast<T>(0)) ? std::optional<T>(t) : std::nullopt;
}

/// @brief Ray-plane intersection.
/// @tparam T Scalar type for coordinates.
/// @param ray Ray to test.
/// @param plane Plane to test.
/// @return Distance along ray to intersection point, or std::nullopt if no intersection.
template<typename T>
auto intersect(const Ray<T>& ray, const Plane<T>& plane) -> std::optional<T>
{
    T denom = plane.normal.dot(ray.direction);

    // Check if ray is parallel to plane
    if (std::abs(denom) < std::numeric_limits<T>::epsilon())
    {
        return std::nullopt;
    }

    // For Hessian form n·x + d = 0, solve for t where n·(o + t*dir) + d = 0
    // => n·o + t*(n·dir) + d = 0  => t = -(n·o + d) / (n·dir)
    T t = -(plane.normal.dot(ray.origin) + plane.d) / denom;

    return (t >= static_cast<T>(0)) ? std::optional<T>(t) : std::nullopt;
}

/// @brief Ray-AABB intersection (slab method).
/// @tparam T Scalar type for coordinates.
/// @param ray Ray to test.
/// @param aabb AABB to test.
/// @return Distance along ray to intersection point, or std::nullopt if no intersection.
template<typename T>
auto intersect(const Ray<T>& ray, const AABB<T>& aabb) -> std::optional<T>
{
    T t_min = static_cast<T>(0);
    T t_max = std::numeric_limits<T>::infinity();

    for (int i = 0; i < 3; ++i)
    {
        T inv_d = static_cast<T>(1) / ray.direction[i];
        T t0 = (aabb.min[i] - ray.origin[i]) * inv_d;
        T t1 = (aabb.max[i] - ray.origin[i]) * inv_d;

        if (inv_d < static_cast<T>(0))
        {
            std::swap(t0, t1);
        }

        t_min = std::max(t_min, t0);
        t_max = std::min(t_max, t1);

        if (t_max < t_min)
        {
            return std::nullopt;
        }
    }

    return (t_min >= static_cast<T>(0)) ? std::optional<T>(t_min) : std::nullopt;
}

/// @brief Sphere-AABB intersection.
/// @tparam T Scalar type for coordinates.
/// @param sphere Sphere to test.
/// @param aabb AABB to test.
/// @return True if they intersect.
template<typename T>
auto intersects(const Sphere<T>& sphere, const AABB<T>& aabb) -> bool
{
    // Find closest point on AABB to sphere center
    Vec<T, 3> closest(microla::clamp(sphere.center.x(), aabb.min.x(), aabb.max.x()),
                      microla::clamp(sphere.center.y(), aabb.min.y(), aabb.max.y()),
                      microla::clamp(sphere.center.z(), aabb.min.z(), aabb.max.z()));

    Vec<T, 3> diff = closest - sphere.center;
    return diff.dot(diff) <= sphere.radius * sphere.radius;
}

// ==================== Triangle ====================

/// @brief Triangle defined by three vertices.
/// @tparam T Scalar type for coordinates.
template<typename T>
struct Triangle
{
    Vec<T, 3> v0, v1, v2;

    /// @brief Default constructor.
    Triangle() : v0(), v1(), v2() {}

    /// @brief Construct triangle from three vertices.
    /// @param a First vertex.
    /// @param b Second vertex.
    /// @param c Third vertex.
    Triangle(const Vec<T, 3>& a, const Vec<T, 3>& b, const Vec<T, 3>& c) : v0(a), v1(b), v2(c) {}

    /// @brief Get triangle normal (counter-clockwise).
    /// @return Unit normal vector.
    [[nodiscard]] auto normal() const -> Vec<T, 3>
    {
        Vec<T, 3> e1 = v1 - v0;
        Vec<T, 3> e2 = v2 - v0;
        return e1.cross(e2).normalized();
    }

    /// @brief Get triangle area.
    /// @return Area of the triangle.
    [[nodiscard]] auto area() const -> T
    {
        Vec<T, 3> e1 = v1 - v0;
        Vec<T, 3> e2 = v2 - v0;
        return e1.cross(e2).length() * static_cast<T>(0.5);
    }

    /// @brief Get centroid.
    /// @return Centroid of the triangle.
    [[nodiscard]] auto centroid() const -> Vec<T, 3>
    {
        return (v0 + v1 + v2) * (static_cast<T>(1) / static_cast<T>(3));
    }

    /// @brief Get axis-aligned bounding box.
    /// @return AABB containing the triangle.
    [[nodiscard]] auto to_aabb() const -> AABB<T>
    {
        AABB<T> box;
        box.expand(v0);
        box.expand(v1);
        box.expand(v2);
        return box;
    }

    /// @brief Check if point is inside triangle (assumes point is on triangle plane).
    /// @param point Point to test.
    /// @return True if point is inside or on the triangle.
    [[nodiscard]] auto contains(const Vec<T, 3>& point) const -> bool
    {
        // Barycentric coordinate method
        Vec<T, 3> v0v1 = v1 - v0;
        Vec<T, 3> v0v2 = v2 - v0;
        Vec<T, 3> v0p = point - v0;

        T dot00 = v0v1.dot(v0v1);
        T dot01 = v0v1.dot(v0v2);
        T dot02 = v0v1.dot(v0p);
        T dot11 = v0v2.dot(v0v2);
        T dot12 = v0v2.dot(v0p);

        T inv_denom = static_cast<T>(1) / (dot00 * dot11 - dot01 * dot01);
        T u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        T v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        return (u >= static_cast<T>(0)) && (v >= static_cast<T>(0)) && (u + v <= static_cast<T>(1));
    }

    /// @brief Ray-triangle intersection.
    /// @param ray Ray to test.
    /// @return Distance along ray to intersection point, or std::nullopt if no intersection.
    [[nodiscard]] auto intersect(const Ray<T>& ray) const -> std::optional<T>
    {
        constexpr T epsilon = std::numeric_limits<T>::epsilon();

        Vec<T, 3> edge1 = v1 - v0;
        Vec<T, 3> edge2 = v2 - v0;

        Vec<T, 3> h = ray.direction.cross(edge2);
        T a = edge1.dot(h);

        // Ray parallel to triangle
        if (std::abs(a) < epsilon)
        {
            return std::nullopt;
        }

        T f = static_cast<T>(1) / a;
        Vec<T, 3> s = ray.origin - v0;
        T u = f * s.dot(h);

        if (u < static_cast<T>(0) || u > static_cast<T>(1))
        {
            return std::nullopt;
        }

        Vec<T, 3> q = s.cross(edge1);
        T v = f * ray.direction.dot(q);

        if (v < static_cast<T>(0) || u + v > static_cast<T>(1))
        {
            return std::nullopt;
        }

        T t = f * edge2.dot(q);

        return (t > epsilon) ? std::optional<T>(t) : std::nullopt;
    }
};

using Trianglef = Triangle<float>;
using Triangled = Triangle<double>;

/// @brief Ray-triangle intersection (Möller-Trumbore algorithm).
/// @tparam T Scalar type for coordinates.
/// @param ray Ray to test.
/// @param tri Triangle to test.
/// @return Tuple (t, u, v) where t is distance along ray and (u, v) are barycentric
///         coordinates, or std::nullopt if no intersection.
template<typename T>
auto intersect(const Ray<T>& ray, const Triangle<T>& tri) -> std::optional<std::tuple<T, T, T>>
{
    constexpr T epsilon = std::numeric_limits<T>::epsilon();

    Vec<T, 3> edge1 = tri.v1 - tri.v0;
    Vec<T, 3> edge2 = tri.v2 - tri.v0;

    Vec<T, 3> h = ray.direction.cross(edge2);
    T a = edge1.dot(h);

    // Ray parallel to triangle
    if (std::abs(a) < epsilon)
    {
        return std::nullopt;
    }

    T f = static_cast<T>(1) / a;
    Vec<T, 3> s = ray.origin - tri.v0;
    T u = f * s.dot(h);

    if (u < static_cast<T>(0) || u > static_cast<T>(1))
    {
        return std::nullopt;
    }

    Vec<T, 3> q = s.cross(edge1);
    T v = f * ray.direction.dot(q);

    if (v < static_cast<T>(0) || u + v > static_cast<T>(1))
    {
        return std::nullopt;
    }

    T t = f * edge2.dot(q);

    if (t > epsilon)
    {
        return std::make_tuple(t, u, v);
    }

    return std::nullopt;
}

// ==================== Frustum ====================

/// @brief View frustum defined by 6 planes.
/// @tparam T Scalar type for coordinates.
template<typename T>
struct Frustum
{
    /// @brief Frustum planes: Left, Right, Bottom, Top, Near, Far.
    std::array<Plane<T>, 6> planes;

    /// @brief Plane index enumeration.
    enum PlaneIndex
    {
        LEFT = 0,
        RIGHT = 1,
        BOTTOM = 2,
        TOP = 3,
        NEAR = 4,
        FAR = 5
    };

    /// @brief Construct frustum from view-projection matrix.
    /// @param vp View-projection matrix.
    /// @return Frustum instance.
    [[nodiscard]] static auto from_matrix(const Mat<T, 4, 4>& vp) -> Frustum
    {
        Frustum f;

        // Extract planes from VP matrix (Gribb-Hartmann method)
        // Left plane
        f.planes[LEFT] =
            Plane<T>(Vec<T, 3>(vp(0, 3) + vp(0, 0), vp(1, 3) + vp(1, 0), vp(2, 3) + vp(2, 0)), vp(3, 3) + vp(3, 0));

        // Right plane
        f.planes[RIGHT] =
            Plane<T>(Vec<T, 3>(vp(0, 3) - vp(0, 0), vp(1, 3) - vp(1, 0), vp(2, 3) - vp(2, 0)), vp(3, 3) - vp(3, 0));

        // Bottom plane
        f.planes[BOTTOM] =
            Plane<T>(Vec<T, 3>(vp(0, 3) + vp(0, 1), vp(1, 3) + vp(1, 1), vp(2, 3) + vp(2, 1)), vp(3, 3) + vp(3, 1));

        // Top plane
        f.planes[TOP] =
            Plane<T>(Vec<T, 3>(vp(0, 3) - vp(0, 1), vp(1, 3) - vp(1, 1), vp(2, 3) - vp(2, 1)), vp(3, 3) - vp(3, 1));

        // Near plane
        f.planes[NEAR] =
            Plane<T>(Vec<T, 3>(vp(0, 3) + vp(0, 2), vp(1, 3) + vp(1, 2), vp(2, 3) + vp(2, 2)), vp(3, 3) + vp(3, 2));

        // Far plane
        f.planes[FAR] =
            Plane<T>(Vec<T, 3>(vp(0, 3) - vp(0, 2), vp(1, 3) - vp(1, 2), vp(2, 3) - vp(2, 2)), vp(3, 3) - vp(3, 2));

        return f;
    }

    /// @brief Check if point is inside frustum.
    /// @param point Point to test.
    /// @return True if inside or on all planes.
    [[nodiscard]] auto contains(const Vec<T, 3>& point) const -> bool
    {
        for (const auto& plane : planes)
        {
            if (plane.distance(point) < static_cast<T>(0))
            {
                return false;
            }
        }
        return true;
    }

    /// @brief Check if sphere is inside frustum.
    /// @param sphere Sphere to test.
    /// @return True if fully inside.
    [[nodiscard]] auto contains(const Sphere<T>& sphere) const -> bool
    {
        for (const auto& plane : planes)
        {
            if (plane.distance(sphere.center) < sphere.radius)
            {
                return false;
            }
        }
        return true;
    }

    /// @brief Check if sphere intersects frustum.
    /// @param sphere Sphere to test.
    /// @return True if intersects or is inside.
    [[nodiscard]] auto intersects(const Sphere<T>& sphere) const -> bool
    {
        for (const auto& plane : planes)
        {
            if (plane.distance(sphere.center) < -sphere.radius)
            {
                return false;
            }
        }
        return true;
    }

    /// @brief Check if AABB intersects frustum.
    /// @param aabb AABB to test.
    /// @return True if intersects or is inside.
    [[nodiscard]] auto intersects(const AABB<T>& aabb) const -> bool
    {
        for (const auto& plane : planes)
        {
            // Find positive vertex (farthest along plane normal)
            Vec<T, 3> p(plane.normal.x() >= static_cast<T>(0) ? aabb.max.x() : aabb.min.x(),
                        plane.normal.y() >= static_cast<T>(0) ? aabb.max.y() : aabb.min.y(),
                        plane.normal.z() >= static_cast<T>(0) ? aabb.max.z() : aabb.min.z());

            if (plane.distance(p) < static_cast<T>(0))
            {
                return false;
            }
        }
        return true;
    }
};

using Frustumf = Frustum<float>;
using Frustumd = Frustum<double>;

}  // namespace geometry
}  // namespace microla
