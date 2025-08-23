#pragma once

#include "geometry.hpp"
#include <cmath>
#include <optional>

namespace geometry::intersections {

template <class... Ts>
struct IntersectionVisitor : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
IntersectionVisitor(Ts...) -> IntersectionVisitor<Ts...>;

inline std::optional<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    const auto visiter = IntersectionVisitor {
        [](const Line &line1, const Line &line2) -> std::optional<Point2D> {
            return { line1.GetIntersectPoint(line2) };
        },
        [](const Circle &circle1, const Circle &circle2) -> std::optional<Point2D> {
            return { circle1.GetIntersectPoint(circle2) };
        },
        [](const Line &line, const Circle &circle) -> std::optional<Point2D> {
            const auto coeffs = line.LineCoeffs();
            if (coeffs) {
                const auto &center = circle.center_p;
                const auto [k, d] = *coeffs;
                const double a = k * k + 1.0;
                const double b = 2.0 * (k * (d - center.y) - center.x);
                const double c = center.x * center.x + (d - center.y) * (d - center.y) - circle.radius * circle.radius;
                const double D = b * b - 4.0 * a * c;

                if (std::abs(D) < eps()) {
                    // D == 0, прямая, на которой лежит отрезок касается окружности
                    const double x = -b / (2.0 * a); // a != 0
                    const double y = k * x + d;
                    const auto p = Point2D{x, y};
                    if (line.ContainsPoint(p)) {
                        return { std::move(p) };
                    }
                }
                else if (D > 0.0) {
                    // D > 0, Линия на которой лежит отрезок дважды пересекает окружность
                    for (const double sign : {-1.0, +1.0}) {
                        const double x = (-b + sign * std::sqrt(D)) / (2.0 * a);
                        const double y = k * x + d;
                        const auto p = Point2D{x, y};
                        if (line.ContainsPoint(p)) {
                            return { std::move(p) };
                        }
                    }
                }
            }
            else {
                // Отрезок лежит на вертикальной линии
                const auto center = circle.center_p;
                if (center.x - circle.radius <= line.start.x && line.start.x <= center.x + circle.radius) {
                    const double x = line.start.x;
                    const double D = circle.radius * circle.radius - (x - center.x) * (x - center.x);
                    if (std::abs(D) < eps()) {
                        // D == 0, Значит вертикальная прямая касается окружности
                        const auto p = Point2D{x, center.y};
                        if (line.ContainsPoint(p)) {
                            return { std::move(p) };
                        }
                    }
                    else if (D > 0.0) {
                        // Две точки пересечения
                        const double y = std::sqrt(D);
                        for (const double sign : {-1.0, +1.0}) {
                            const auto p = Point2D{x, center.y + sign * y};
                            if (line.ContainsPoint(p)) {
                                return { std::move(p) };
                            }
                        }
                    }
                    else {
                        throw std::logic_error("Vertical line do not intersect circle");
                    }
                }
            }
            return std::nullopt;
        },
        [](const auto &shape1, const auto &shape2) -> std::optional<Point2D> {
            throw std::logic_error("Unsupported shapes combination");
            return std::nullopt;
        }
    };
    return std::visit(visiter, shape1, shape2);
}

}  // namespace geometry::intersections
