#pragma once

#include "geometry.hpp"
#include "queries.hpp"
#include <print>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0, double max_size = 20.0)
        : gen(20), coord_dist(min_coord, max_coord), size_dist(min_size, max_size), sides_dist(3, 12), type_dist(0, 4) {
    }

    Shape GenerateRandomShape() {
        Point2D center{coord_dist(gen), coord_dist(gen)};
        double size = size_dist(gen);

        switch (type_dist(gen)) {
            case 0: {
                Point2D end{center.x + size, center.y + size};
                return Line{center, end};
            }
            case 1: {
                Point2D a{center.x, center.y};
                Point2D b{center.x + size, center.y};
                Point2D c{center.x + size / 2, center.y + size};
                return Triangle{a, b, c};
            }
            case 2: {
                return Rectangle{center, size, size * 0.8};
            }
            case 3: {
                int sides = sides_dist(gen);
                return RegularPolygon{center, size, sides};
            }
            case 4: {
                return Circle{center, size};
            }
        }
        return Circle{center, size};
    }

    std::vector<Shape> GenerateShapes(size_t count) {
        std::vector<Shape> shapes;
        shapes.reserve(count);

        while (count --> 0) {
            shapes.emplace_back(GenerateRandomShape());
        }

        return shapes;
    }

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

std::vector<std::pair<Shape, Shape>> FindAllCollisions(const std::vector<Shape> &shapes) {
    return std::views::cartesian_product(shapes, shapes)
        | std::views::filter([](const auto &p){
            const auto visiter = [](auto &&shape1, auto &&shape2) {
                const auto bbox1 = shape1.GetBoundingBox();
                const auto bbox2 = shape2.GetBoundingBox();
                if (bbox1 == bbox2) {
                    return false;
                } else {
                    return bbox1.Overlaps(bbox2);
                }
            };
            return std::visit(visiter, std::get<0>(p), std::get<1>(p));
        })
        | std::ranges::to<std::vector<std::pair<Shape, Shape>>>();
}

std::optional<Shape> FindHighestShape(const std::vector<Shape> &shapes) {
    if (shapes.size() == 0) {
        return std::nullopt;
    } else {
        const auto height_lambda = [](const auto &v) -> double { return v.Height(); };
        const auto height_comp = [&height_lambda](const auto &lhd, const auto &rhd) -> bool {
            return lhd.visit(height_lambda) < rhd.visit(height_lambda);
        };
        return { *std::ranges::max_element(shapes, height_comp) };
    }
}

}  // namespace geometry::utils
