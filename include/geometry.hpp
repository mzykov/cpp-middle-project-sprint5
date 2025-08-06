#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <expected>
#include <format>
#include <numbers>
#include <numeric>
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
    using ValueType = double;
    const ValueType x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(ValueType x, ValueType y) : x(x), y(y) {}

    // Comparison
    constexpr inline bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    constexpr inline bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }

    // Binary math operators
    constexpr inline Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    constexpr inline Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    constexpr inline Point2D operator*(double value) const { return {x * value, y * value}; }
    constexpr inline Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    constexpr inline double Dot(const Point2D &other) const { return x * other.x + y * other.y; }
    constexpr inline double Cross(const Point2D &other) const { return x * other.y - y * other.x; }
    constexpr inline double Length() const { return std::sqrt(x * x + y * y); }
    constexpr inline double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    constexpr inline Point2D Normalize() const {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <size_t N>
struct Lines2D {
    const std::array<double, N> x;
    const std::array<double, N> y;
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
    constexpr inline Point2D Front() const { return {x.front(), y.front()}; }
};

struct BoundingBox {
    const Point2D left_bottom, right_top;

    BoundingBox() = default;
    BoundingBox(const Point2D &lb, const Point2D &rt) :
        left_bottom(lb < rt ? lb : rt),
        right_top(lb < rt ? rt : lb) {
            if (left_bottom.x == right_top.x || left_bottom.y == right_top.y) {
                throw std::invalid_argument("Degenerated bounding box\n");
            }
        }

    constexpr inline bool Overlaps(const BoundingBox &other) const {
        if (Left() >= other.Right() || other.Left() >= Right()) {
            return false;
        } else if (Top() <= other.Bottom() || Bottom() >= other.Top()) {
            return false;
        } else {
            return true;
        }
    }

    constexpr inline bool Contains(const BoundingBox &other) const {
        return Left()        <= other.Left()
            && other.Right() <= Right()
            && Bottom()      <= other.Bottom()
            && other.Top()   <= Top();
    }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        return left_bottom < p && p < right_top;
    }

    constexpr inline Point2D::ValueType Left()   const { return left_bottom.x; }
    constexpr inline Point2D::ValueType Right()  const { return right_top.x; }
    constexpr inline Point2D::ValueType Bottom() const { return left_bottom.y; }
    constexpr inline Point2D::ValueType Top()    const { return right_top.y; }
    constexpr inline Point2D::ValueType Width()  const { return (right_top - left_bottom).x; }
    constexpr inline Point2D::ValueType Height() const { return (right_top - left_bottom).y; }

    constexpr inline Point2D LeftBottom()  const { return left_bottom; }
    constexpr inline Point2D RightTop()    const { return right_top; }
    constexpr inline Point2D LeftTop()     const { return Point2D{left_bottom.x, right_top.y}; }
    constexpr inline Point2D RightBottom() const { return Point2D{right_top.x, left_bottom.y}; }
    constexpr inline Point2D Center()      const { return (right_top - left_bottom) / 2.0; }
};

struct Line {
    const Point2D start, end;

    constexpr inline Point2D::ValueType Length() const { return start.DistanceTo(end); }
    constexpr inline Point2D::ValueType Height() const { return std::max(start.y, end.y); }
    constexpr inline BoundingBox GetBoundingBox() const { return BoundingBox(start, end); }
    constexpr inline Point2D Center() const { return (end - start) / 2.0; }
    constexpr inline std::array<Point2D, 2> Vertices() const { return {start, end}; }
    constexpr inline Lines2D<2> Lines() const { return {{start.x, end.x}, {start.y, end.y}}; }
    constexpr inline Point2D Direction() const { return (end - start).Normalize(); }
};

struct Triangle {
    const Point2D a, b, c;

    constexpr inline double Area() const {
        return (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0;
    }

    constexpr inline Point2D Center() const { return (a + b + c) / 3.0; }

    constexpr inline BoundingBox GetBoundingBox() const {
        return {
            { std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}) },
            { std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}) }
        };
    }

    constexpr inline double Height() const { return std::max({ a.y, b.y, c.y }); }

    constexpr inline std::array<Point2D, 3> Vertices() const { return {a, b, c}; }
    constexpr inline Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }
};

struct Rectangle {
    const Point2D left_bottom;
    double width, height;

    constexpr inline double Area() const { return width * height; }
    constexpr inline Point2D Center() const { return left_bottom + Point2D{width/2.0, height/2.0}; }
    constexpr inline BoundingBox GetBoundingBox() const { return {left_bottom, left_bottom + Point2D{width, height}}; }
    constexpr inline double Height() const { return left_bottom.y + height; }
    constexpr inline double Width() const { return width; }
    constexpr inline std::array<Point2D, 1> Vertices() const { return {{ left_bottom }}; }
    constexpr inline Lines2D<1> Lines() const { return {{left_bottom.x}, {left_bottom.y}}; }
};

struct RegularPolygon {
    const Point2D center_p;
    const double radius;
    const int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    constexpr inline double Area() const {
        return sides * radius * radius * sin(2.0 * std::numbers::pi / sides) / 2.0;
    }

    constexpr inline BoundingBox GetBoundingBox() const {
        return {
            { center_p.x - radius, center_p.y - radius },
            { center_p.x + radius, center_p.y + radius }
        };
    }

    constexpr inline Point2D Center() const { return center_p; }
    constexpr inline double Height() const { return center_p.y + radius; }

    std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2.0 * std::numbers::pi * i / sides;
            points.emplace_back(
                center_p.x + radius * std::cos(angle),
                center_p.y + radius * std::sin(angle)
            );
        }
        return points;
    }

    Lines2DDyn Lines() const {
        auto vertices = Vertices();
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }
};

struct Circle {
    const Point2D center_p;
    const double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    constexpr inline BoundingBox GetBoundingBox() const {
        return {
            { center_p.x - radius, center_p.y - radius },
            { center_p.x + radius, center_p.y + radius }
        };
    }

    constexpr inline double Height() const { return center_p.y + radius; }
    constexpr inline Point2D Center() const { return center_p; }

    std::vector<Point2D> Vertices(size_t N = 30) const {
        std::vector<Point2D> points;
        points.reserve(N);

        for (int i = 0; i < N; ++i) {
            const double angle = 2.0 * std::numbers::pi * i / N;
            points.emplace_back(
                center_p.x + radius * std::cos(angle),
                center_p.y + radius * std::sin(angle)
            );
        }
        return points;
    }

    Lines2DDyn Lines(size_t N = 100) const {
        auto vertices = Vertices(N);
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }
};

class Polygon {
public:
    std::vector<Point2D> vertices;

    constexpr inline Point2D Center() const {
        double center_x = 0, center_y = 0;
        for (const auto &v : vertices) {
            center_x += v.x;
            center_y += v.y;
        }
        return Point2D{center_x, center_y} / vertices.size();
    }

    constexpr inline double Height() const {
        return (*std::max_element(vertices.begin(), vertices.end(),
            [](const auto &lhd, const auto &rhd) {
                return lhd.y < rhd.y;
            })).y;
    }

    constexpr inline BoundingBox GetBoundingBox() const {
        const auto [min_x, max_x] = std::minmax_element(vertices.begin(), vertices.end(),
            [](const auto &lhd, const auto &rhd) {
                return lhd.x < rhd.x;
            }
        );
        const auto [min_y, max_y] = std::minmax_element(vertices.begin(), vertices.end(),
            [](const auto &lhd, const auto &rhd) {
                return lhd.y < rhd.y;
            }
        );
        return { {(*min_x).x, (*min_y).y}, {(*max_x).x, (*max_y).y} };
    }

    std::vector<Point2D> Vertices() const { return vertices; }

    Lines2DDyn Lines() const {
        auto vertices = Vertices();
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

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

    constexpr auto parse(std::format_parse_context &ctx) {
        auto it = ctx.begin();
        std::string marker;
        while (it != ctx.end()) {
            marker = marker + *it;
        }
        if (marker == "new_line") {
            use_new_line = true;
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        for (const auto &p : v) {
            if (use_new_line) {
                std::format_to(ctx.out(), "\n\t");
            }
            std::format_to(ctx.out(), "{} ", p);
        }
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
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.left_bottom, r.width,
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
