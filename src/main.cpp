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

namespace rng = std::ranges;
namespace views = std::ranges::views;

void PrintAllIntersections(const Shape &shape, const std::vector<Shape> &others) {
    std::println("\n=== Intersections ===");

    /*
     * Используйте ranges чтобы оставить только фигуры,
     * поддерживающие возможность находить пересечения между собой
     *
     * Затем примените монадический интерфейс для обработки результатов:
     *     - Пересечение найдено в точке A между фигурами B и C
     *     - Фигуры B и C не пересекаются
     */
}

void PrintDistancesFromPointToShapes(Point2D p, const std::vector<Shape> &shapes) {
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

void PerformShapeAnalysis(ReplaceMe shapes) {
    std::println("\n=== Shape Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Найти все пересечения между фигурами используя метод Bounding Box
     *     - Найти самую высокую фигуру (чья высота наибольшая)
     *     - Вывести расстояние между любыми двумя фигурами, которые поддерживают данную функциональность
     */
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    /*
     * Используйте ranges и созданные классы чтобы:
     *     - Вывести 3 любые фигуры, которые находятся выше 50.0
     *     - Вывести фигуры с наименьшей и с наибольшей высотами
     */
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector<Shape> shapes = generator.GenerateShapes(15);

    std::println("Generated {} random shapes", shapes.size());

    for (size_t i = 0; i < shapes.size(); ++i) {
        const auto &shape = shapes[i];
        shape.visit([i](auto &&v){
            std::println("Фигура №{} имеет высоту {:.4}", i, v.Height());
        });
    }

    //
    // Вызываем разработанные функции
    //
    PrintAllIntersections(shapes[0], shapes);
    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);
    PerformShapeAnalysis(shapes);
    PerformExtraShapeAnalysis(shapes);

    //
    // Рисуем все фигуры
    //
    // Важно: после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 2ой график
    //
    geometry::visualization::Draw(shapes);

    //
    // Формируем список из вершин всех фигур
    //
    std::vector<Point2D> points;

    /* ваш код здесь */

    //
    // Находим список точек, для построения выпуклой оболочки - convex hull - алгоритмом Грэхема
    // Создаём из них объект класса `Polygon` и добавляем его в список shapes
    // Рисуем все фигуры
    //

    /* ваш код здесь */

    //
    // после изучения графика - нажмите Enter чтобы продолжить выполнение и построить 3ий график
    //

    {
        std::vector<Point2D> points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};

        //
        // Используйте список точек points или свой, чтобы
        // выполнить алгоритм триангуляции Делоне алгоритмом Боуэра-Ватсона
        // После успешного завершения алгоритма - выведите результат для проверки
        // используя geometry::visualization::Draw
        //
    }

    return 0;
}
