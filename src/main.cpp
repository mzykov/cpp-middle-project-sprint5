#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <print>
#include <random>
#include <ranges>
#include <unordered_set>

using namespace geometry;

void PrintAllIntersections(const Shape &shape, std::span<const Shape> others) {
    std::println("\n=== Intersections ===");

    for (const auto &v : others) {
        const auto maybe_intersection = intersections::GetIntersectPoint(shape, v);
        if (!maybe_intersection.has_value()) {
            continue;
        }

        const auto optional_point = maybe_intersection.value();

        std::visit([&optional_point](const auto &v1, const auto &v2) {
            if (optional_point) {
                std::println("Пересечение найдено в точке {} между фигурами {} и {}", *optional_point, v1, v2);
            } else {
                std::println("Фигуры {} и {} не пересекаются", v1, v2);
            }
        }, shape, v);
    }
}

void PrintDistancesFromPointToShapes(const Point2D &p, std::span<const Shape> shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", p);

    if (shapes.size() == 0) {
        return;
    }

    constexpr size_t sz = 5;
    const size_t num_shapes = std::min(sz, shapes.size());

    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_int_distribution<size_t> distrib(0, shapes.size() - 1);
    std::unordered_set<size_t> already;

    for (size_t i = 0; i < num_shapes; ++i) {
        size_t index = distrib(g);
        while (already.contains(index)) {
            index = distrib(g);
        }
        already.insert(index);
        const auto shape = shapes[index];
        const auto dist = queries::DistanceToPoint(shape, p);
        shape.visit([&p, &dist](auto &&v){
            std::println("Расстояние от точки {} до фигуры {} равно {:.4}", p, v, dist);
        });
    }
}

void PerformShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Analysis ===");
    {
        const auto collisions = utils::FindAllCollisions(shapes);
        for (const auto &[shape1, shape2] : collisions) {
            std::visit([](const auto &v1, const auto &v2) {
                std::println("Ограничивающие прямоугольники фигур {} и {} перекрываются", v1, v2);
            }, shape1, shape2);
        }
    }
    {
        const auto highestShape = utils::FindHighestShape(shapes);
        if (highestShape) {
            (*highestShape).visit([](auto &&v){
                std::println("Фигура с наибольшей *высотой* {} (Y = {:.4})", v, v.Height());
            });
        }
    }
    {
        auto pairs_view = std::views::cartesian_product(shapes, shapes)
            | std::views::filter([](const auto &p){
                return std::get<0>(p) != std::get<1>(p);
            });

        for (const auto &[shape1, shape2] : pairs_view) {
            const auto maybe_dist = queries::DistanceBetweenShapes(shape1, shape2);
            if (!maybe_dist.has_value()) {
                continue;
            }
            std::visit([&maybe_dist](const auto &v1, const auto &v2) {
                std::println("Расстояние между фигурами {} и {} равно {:.4}", v1, v2, maybe_dist.value());
            }, shape1, shape2);
        }
    }
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");
    {
        constexpr size_t num = 3;
        constexpr double min_height = 50.0;
        auto shapes_view = shapes
            | std::views::filter([min_height](auto &&shape){
                return shape.visit([](auto &&v){ return v.Height(); }) > min_height;
            })
            | std::views::take(num);

        for (const auto &shape : shapes_view) {
            shape.visit([min_height](auto &&v){
                std::println("Фигура {} находится выше Y-координаты {}", v, min_height);
            });
        }
    }
    {
        const auto highestShape = utils::FindHighestShape(shapes);
        if (highestShape) {
            (*highestShape).visit([](auto &&v){
                std::println("Фигура с наибольшей *высотой* {} (Y = {:.4})", v, v.Height());
            });
        }
    }
    {
        const auto lowestShape = utils::FindLowestShape(shapes);
        if (lowestShape) {
            (*lowestShape).visit([](auto &&v){
                std::println("Фигура с наименьшей *высотой* {} (Y = {:.4})", v, v.Height());
            });
        }
    }
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector<Shape> shapes = generator.GenerateShapes(15);

    {
        std::println("Generated {} random shapes", shapes.size());

        for (size_t i = 0; i < shapes.size(); ++i) {
            const auto &shape = shapes[i];
            shape.visit([i](auto &&v){
                std::println("Фигура №{} имеет высоту {:.4}", i, v.Height());
            });
        }
    }

    //
    // Вызываем разработанные функции
    //
    {
        PrintAllIntersections(shapes[0], {std::next(shapes.begin()), shapes.end()});
        PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);
        PerformShapeAnalysis(shapes);
        PerformExtraShapeAnalysis(shapes);
        visualization::Draw(shapes);
    }

    //
    // Формируем список из вершин всех фигур.
    // Находим список точек, для построения выпуклой оболочки - convex hull - алгоритмом Грэхема.
    // Создаём из них объект класса `Polygon` и добавляем его в список shapes.
    // Рисуем все фигуры.
    //
    {
        auto points = shapes
            | std::views::transform([](auto &&shape){
                return shape.visit([](auto &&v){ return v.Vertices(); });
            })
            | std::views::join
            | std::ranges::to<std::vector>();
        const auto conv = convex_hull::GrahamScan(points);

        if (conv.has_value()) {
            shapes.emplace_back(Polygon{conv.value()});
            visualization::Draw(shapes);
        } else {
            //throw std::logic_error("Cannot construct convex hull\n");
        }
    }

    //
    // после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 3-ий график
    //
    {
        std::span<const Point2D> points {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};
        const auto tri = triangulation::DelaunayTriangulation(points);

        if (tri.has_value()) {
            std::vector<Shape> tshapes {tri.value().begin(), tri.value().end()};
            visualization::Draw(tshapes);
        } else {
            throw std::logic_error("Cannot construct Delaunay triangulation\n");
        }
    }

    return 0;
}
