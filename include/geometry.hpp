#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <format>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <variant>
#include <vector>

namespace geometry {

/*
 * Добавьте к методам класса Point2D и Lines2DDyn все необходимые аттрибуты и спецификаторы
 * Важно: Возвращаемый тип и принимаемые аргументы менять не нужно
 */
struct Point2D {
    const double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    constexpr bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    constexpr bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }

    // Binary math operators
    constexpr Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    constexpr Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    constexpr Point2D operator*(double value) const { return {x * value, y * value}; }
    constexpr Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    constexpr double Dot(const Point2D &other) const { return x * other.x + y * other.y; }
    constexpr double Cross(const Point2D &other) const { return x * other.y - y * other.x; }
    constexpr double Length() const { return std::sqrt(x * x + y * y); }
    constexpr double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    constexpr Point2D Normalize() const {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <size_t N>
struct Lines2D {
    std::array<double, N> x;
    std::array<double, N> y;
};

struct Lines2DDyn {
    std::vector<double> x;
    std::vector<double> y;

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
    }
    void PushBack(Point2D p) {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    void PushBack(double px, double py) {
        x.push_back(px);
        y.push_back(py);
    }
    Point2D Front() { return {x.front(), y.front()}; }
};

struct BoundingBox {
    const Point2D left_bottom, right_top;

    constexpr inline bool Overlaps(const BoundingBox &other) const {
        return true;
    }
    constexpr inline Point2D Center() const { return (right_top - left_bottom) / 2.0; }
    constexpr inline double Width() const { return (right_top - left_bottom).x; }
    constexpr inline double Height() const { return (right_top - left_bottom).y; }
};

struct Line {
    const Point2D start, end;

    /* ваш код здесь */

    constexpr Point2D Center() const { return {}; }
    constexpr std::array<Point2D, 2> Vertices() const { return {Point2D{start.x, start.y}, {end.x, end.y}}; }
    constexpr Lines2D<2> Lines() const { return {{start.x, end.x}, {start.y, end.y}}; }
};

struct Triangle {
    const Point2D a, b, c;

    //
    // Обратите внимание! В методе Lines(), в отличие от Vertices(), координаты точек замыкаются на начало:
    // a.x, b.x, c.x а затем идёт снова первая вершина a.x
    //
    // Это необходимо для правильного рисования фигур через gnuplot, который формирует линии используя пары точек.
    // В случае с  Triangle будут составлены такие пары точек:
    //      - { a, b }
    //      - { b, c }
    //      - { c, a }
    //
    constexpr Point2D Center() const { return {}; }
    constexpr std::array<Point2D, 3> Vertices() const { return {a, b, c}; }
    constexpr Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }

    /* ваш код здесь */
};

struct Rectangle {
    const Point2D bottom_left;
    double width, height;

    /* ваш код здесь */
    constexpr Point2D Center() { return {}; }
    constexpr std::array<Point2D, 1> Vertices() { return {}; }
    constexpr Lines2D<1> Lines() const { return {}; }
};

struct RegularPolygon {
    const Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    constexpr Point2D Center() const { return {}; }
    std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }
    Lines2DDyn Lines() const { return {}; }
};

struct Circle {
    const Point2D center_p;
    const double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    constexpr BoundingBox BoundBox() const {
        return {
            { center_p.x - radius, center_p.y - radius },
            { center_p.x + radius, center_p.y + radius }
        };
    }
    constexpr double Height() const { return center_p.y + radius; }
    constexpr Point2D Center() const { return center_p; }

    //
    // Должны быть сделана по аналогии с RegularPolygon::Vertices
    //
    std::vector<Point2D> Vertices(size_t N = 30) { return {}; }
    Lines2DDyn Lines(size_t N = 100) const { return {}; }
};

class Polygon {
public:
    /* ваш код здесь */

    //
    // Должны быть сделана по аналогии с RegularPolygon::Vertices
    //
    constexpr Point2D Center() const { return {}; }
    std::vector<Point2D> Vertices(size_t N = 30) const { return {}; }
    Lines2DDyn Lines(size_t N = 100) const { return {}; }

private:
    std::vector<Point2D> points_;
    BoundingBox bounding_box_;
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

/*
 * В коде везде используется ReplaceMe. Ваша задача - удалить ReplaceMe и везде вместо него
 использовать наиболее подходящий тип для решения задачи
 */
struct ReplaceMe {
    ReplaceMe(std::vector<Shape>) {}
};

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Point2D &p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};
template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) const {
        auto it = ctx.begin();

        /* ваш код здесь */

        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {

        /* ваш код здесь */
        return ctx.out();
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};
