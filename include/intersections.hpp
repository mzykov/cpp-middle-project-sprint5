#pragma once

#include "geometry.hpp"
#include <cmath>
#include <optional>

namespace geometry::intersections {

/*
 * Класс для поиска пересечений между двумя фигурами
 *
 * Требуется организовать возможность нахождения пересечений только для следующих комбинаций фигур:
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется вернуть std::nullopt
 */
class IntersectionVisitor {
public:
    IntersectionVisitor(const Line &l1, const Line &l2) {}
};

inline std::optional<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    return std::nullopt;
}

}  // namespace geometry::intersections
