#pragma once

#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <set>
#include <vector>

namespace geometry::triangulation {

inline Triangle GetDelaunaySuperTriangle(std::span<const Point2D> points) {
    // Находим "центр тяжести" системы точек.
    // Эта точка будет центром окружности, которая будет вписана в супертреугольник.
    // Т.к. эта окружность содержит внутри себя все точки, значит супертреугольник
    // также содержит все исходные точки
    const auto super_center = Polygon(std::vector(points.begin(), points.end())).Center();
    const auto farthes_point = *std::ranges::max_element(points, [super_center](const auto &lhs, const auto &rhs){
        return super_center.DistanceTo(lhs) < super_center.DistanceTo(rhs);
    });
    // Радиус берём с запасом 2*eps()
    const double super_radius = super_center.DistanceTo(farthes_point) + 2.0 * eps();
    // Половина длины стороны равностороннего супертреугольника равна
    const double super_edge_semilength = std::sqrt(3.0) * super_radius;

    return Triangle{
        Point2D{super_center.x - super_edge_semilength, super_center.y - super_radius},
        Point2D{super_center.x + super_edge_semilength, super_center.y - super_radius},
        Point2D{super_center.x, super_center.y + 2.0 * super_radius}
    };
}

inline bool PointViolatesDelaunayConditionForTriangle(const Point2D &p, const Triangle &t) {
    return t.CircumCircleContainsPoint(p);
}

inline GeometryResult<std::vector<Triangle>> DelaunayTriangulation(std::span<const Point2D> points) {
    if (points.size() < 3) {
        return std::unexpected(GeometryError::InsufficientPoints);
    }

    // Создаём вектор для хранения текущей триангуляции и добавляем в него
    // "Супертреугольник", содержащий внутри себя все точки
    std::vector<Triangle> triangulation;
    const auto super_triangle = GetDelaunaySuperTriangle(points);
    triangulation.push_back(super_triangle);

    for (const auto &p : points) {
        std::set<Line> polygonal_hole;
        std::vector<Triangle> next_triangulation;

        while (!triangulation.empty()) {
            auto t = triangulation.back();
            triangulation.pop_back();

            if (PointViolatesDelaunayConditionForTriangle(p, t)) {
                for (const auto &face : t.GetFaces()) {
                    auto already_it = polygonal_hole.find(face);
                    if (already_it == polygonal_hole.end()) {
                        polygonal_hole.insert(face);
                    } else {
                        polygonal_hole.erase(already_it);
                    }
                }
            } else {
                next_triangulation.push_back(std::move(t));
            }
        }

        for (const auto &l : polygonal_hole) {
            next_triangulation.emplace_back(l.start, l.end, p);
        }

        triangulation.swap(next_triangulation);
    }

    std::erase_if(triangulation,
        [&super_triangle](const auto &t) {
            return t.SharesVertex(super_triangle);
        }
    );

    return triangulation;
}

}  // namespace geometry::triangulation
