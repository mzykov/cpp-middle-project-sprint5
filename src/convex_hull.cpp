#include "convex_hull.hpp"
#include <algorithm>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}

GeometryResult<std::vector<Point2D>> GrahamScan(std::vector<Point2D> &points) {
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InvalidInput};
    }

    const auto p0 = *std::ranges::min_element(points, [](const auto &a, const auto &b) {
        return std::pair(a.y, a.x) < std::pair(b.y, b.x);
    });

    std::ranges::sort(points, [&p0](const Point2D &a, const Point2D &b) {
        const auto crossp = CrossProduct(a, p0, b);

        if (std::abs(crossp) < eps()) {
            return p0.DistanceTo(a) < p0.DistanceTo(b);
        }

        return crossp < 0.0;
    });

    StackForGrahamScan stack;

    for (const auto &next_point : points) {
        while (stack.Size() > 1 && CrossProduct(stack.NextToTop(), stack.Top(), next_point) <= 0.0) {
            stack.Pop();
        }
        stack.Push(next_point);
    }

    if (stack.Size() >= 3) {
        return std::move(stack).Extract();
    } else {
        return std::unexpected{GeometryError::InsufficientPoints};
    }
}

}  // namespace geometry::convex_hull
