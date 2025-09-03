#pragma once

#include "geometry.hpp"

namespace geometry::intersections {

template <class... Ts>
struct IntersectionVisitor : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
IntersectionVisitor(Ts...) -> IntersectionVisitor<Ts...>;

inline GeometryResult<std::optional<Point2D>> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    const auto visiter = IntersectionVisitor {
        [](const Line &line1, const Line &line2) -> GeometryResult<std::optional<Point2D>> {
            return line1.GetIntersectPoint(line2);
        },
        [](const Circle &circle1, const Circle &circle2) -> GeometryResult<std::optional<Point2D>> {
            return circle1.GetIntersectPoint(circle2);
        },
        [](const Line &line, const Circle &circle) -> GeometryResult<std::optional<Point2D>> {
            return circle.GetIntersectPoint(line);
        },
        [](const Circle &circle, const Line &line) -> GeometryResult<std::optional<Point2D>> {
            return circle.GetIntersectPoint(line);
        },
        [](const auto &shape1, const auto &shape2) -> GeometryResult<std::optional<Point2D>> {
            return std::unexpected{GeometryError::Unsupported};
        }
    };
    return std::visit(visiter, shape1, shape2);
}

}  // namespace geometry::intersections
