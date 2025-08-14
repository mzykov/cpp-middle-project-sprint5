#pragma once

#include "geometry.hpp"
#include <algorithm>
#include <optional>
#include <variant>

namespace geometry::queries {

struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double accept(Line &&line) const {
        const auto coeffs = line.LineCoeffs();
        if (coeffs) {
            const auto [k, b] = *coeffs;
            const double x = (point.x + k * (point.y - b)) / (k * k + 1.0);
            const double y = k * x + b;
            const auto p = Point2D{x, y};
            if (line.ContainsPoint(p)) {
                return point.DistanceTo(p);
            }
        }
        else {
            // Прямая x = Const
            // Перпендикуляр пересекает её в точке {Const, point.y}
            const auto p = Point2D{line.start.x, point.y};
            if (line.ContainsPoint(p)) {
                return std::abs(point.x - line.start.x);
            }
        }
        return std::min(point.DistanceTo(line.start), point.DistanceTo(line.end));
    }

    double accept(Circle &&circle) const {
        const auto dist = point.DistanceTo(circle.center_p);
        if (dist > circle.radius) {
            const auto intersection = intersections::GetIntersectPoint(circle, Line{circle.center_p, point});
            if (intersection) {
                return Line{*intersection, point}.Length();
            } else {
                throw std::logic_error("No intersection between point and circle");
            }
        } else {
            return 0.0;
        }
    }

    double accept(auto &&v) const {
        if (v.ContainsPoint(point)) {
            return 0.0;
        } else {
            const auto faces = v.GetFaces();
            const auto dists = faces
                | std::views::transform([&](const auto &face){ return accept(face); })
                | std::ranges::to<std::vector>();
            return *std::min_element(dists.begin(), dists.end());
        }
    }
};

template <class... Ts>
struct ShapeToShapeDistanceVisitor : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
ShapeToShapeDistanceVisitor(Ts...) -> ShapeToShapeDistanceVisitor<Ts...>;

inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    const auto visiter = PointToShapeDistanceVisitor{point};
    return shape.visit([&](auto &&v){ return visiter.accept(v); });
}

inline BoundingBox GetBoundingBox(const Shape &shape) {
    return shape.visit([](auto &&v){ return v.GetBoundingBox(); });
}

inline double GetHeight(const Shape &shape) {
    return shape.visit([](auto &&v){ return v.Height(); });
}

inline bool BoundingBoxesOverlaps(const Shape &shape1, const Shape &shape2) {
    const auto bbox1 = shape1.visit([](auto &&v){ return v.GetBoundingBox(); });
    const auto bbox2 = shape2.visit([](auto &&v){ return v.GetBoundingBox(); });
    return bbox1.Overlaps(bbox2);
}

std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    const auto visiter = ShapeToShapeDistanceVisitor {
        [](const Line &line1, const Line &line2) -> std::optional<double> {
            if (line1.SharesSameLine(line2) && line1.Overlaps(line2)) {
                return { 0.0 };
            }
            else if (line1.IsParallelTo(line2) && line1.Overlaps(line2)) {
                const auto data1 = line1.LineCoeffs();
                const auto data2 = line2.LineCoeffs();
                if (data1 && data2) {
                    const auto [k1, b1] = *data1;
                    const auto [k2, b2] = *data2;
                    if (std::abs(k1) < eps() && std::abs(k2) < eps()) {
                        // Две прямые y = Const1 и y = Const2
                        return { std::abs(b2 - b1) };
                    }
                    else if (std::abs(k1) > eps() && std::abs(k2) > eps()) {
                        const auto dist = std::abs(
                            ((b2 - b1) * k1 * k2 / (k1 * b2 - k2 * b1))
                            /
                            std::sqrt(
                                (b2 - b1) * (b2 - b1) + ((k1 * b2 - k2 * b1) * (k1 * b2 - k2 * b1)) / (k1 * k1 * k2 * k2)
                            )
                        );
                        return { dist };
                    }
                    else {
                        throw std::logic_error("Angle coefficients are differ, but lines must be parallel");
                    }
                }
                else if (!data1 && !data2) {
                    // Две прямые x = Const1 и x = Const2
                    return { std::abs(line1.start.x - line2.start.x) };
                }
                else {
                    throw std::logic_error("Lines must be parallel, but they are not");
                }
                return {};
            }
            else {
                const auto ss = line1.start.DistanceTo(line2.start);
                const auto se = line1.start.DistanceTo(line2.end);
                const auto es = line1.end.DistanceTo(line2.start);
                const auto ee = line1.end.DistanceTo(line2.end);
                return { std::min({ ss, se, es, ee }) };
            }
        },
        [](const Circle &circle1, const Circle &circle2) -> std::optional<double> {
            const auto l = Line{circle1.center_p, circle2.center_p};

            if (l.Length() > circle1.radius + circle2.radius) {
                const auto i1 = intersections::GetIntersectPoint(circle1, l);
                const auto i2 = intersections::GetIntersectPoint(circle2, l);
                if (i1 && i2) {
                    return { Line{*i1, *i2}.Length() };
                } else {
                    return std::nullopt;
                }
            } else {
                return { 0.0 };
            }
        },
        [](const auto &v1, const auto &v2) -> std::optional<double> {
            return std::nullopt;
        },
    };
    return std::visit(visiter, shape1, shape2);
}

}  // namespace geometry::queries
