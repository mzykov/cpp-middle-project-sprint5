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
#include <random>
#include <ranges>
#include <span>
#include <variant>
#include <vector>

namespace geometry {

constexpr inline double eps() { return 1e-10; }

struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    constexpr inline bool operator==(const Point2D &other) const { return std::abs(x - other.x) < eps() && std::abs(y - other.y) < eps(); }
    constexpr inline bool operator!=(const Point2D &other) const { return !(*this == other); }
    constexpr inline bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    constexpr inline bool operator<=(const Point2D &other) const { return x <= other.x && y <= other.y; }

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

    constexpr BoundingBox() = default;
    constexpr BoundingBox(const Point2D &lb, const Point2D &rt) :
        left_bottom(lb < rt ? lb : rt),
        right_top(lb < rt ? rt : lb) {
            if (left_bottom.x == right_top.x || left_bottom.y == right_top.y) {
                throw std::invalid_argument("Degenerated bounding box\n");
            }
        }

    constexpr inline bool operator==(const BoundingBox &other) const = default;
    constexpr inline bool operator!=(const BoundingBox &other) const = default;

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
        return left_bottom <= p && p <= right_top;
    }

    constexpr inline double Left()   const { return left_bottom.x; }
    constexpr inline double Right()  const { return right_top.x; }
    constexpr inline double Bottom() const { return left_bottom.y; }
    constexpr inline double Top()    const { return right_top.y; }
    constexpr inline double Width()  const { return (right_top - left_bottom).x; }
    constexpr inline double Height() const { return (right_top - left_bottom).y; }

    constexpr inline Point2D LeftBottom()  const { return left_bottom; }
    constexpr inline Point2D RightTop()    const { return right_top; }
    constexpr inline Point2D LeftTop()     const { return Point2D{left_bottom.x, right_top.y}; }
    constexpr inline Point2D RightBottom() const { return Point2D{right_top.x, left_bottom.y}; }
    constexpr inline Point2D Center()      const { return left_bottom + ((right_top - left_bottom) / 2.0); }
};

struct Line {
    const Point2D start, end;

    constexpr Line(const Point2D &s, const Point2D &e) :
        start(s.x < e.x ? s : s.x > e.x ? e : s.y <= e.y ? s : e),
        end(s.x < e.x ? e : s.x > e.x ? s : s.y <= e.y ? e : s) {}

    constexpr inline bool operator==(const Line &other) const = default;
    constexpr inline bool operator!=(const Line &other) const = default;

    constexpr inline bool operator<(const Line &other) const {
        if (std::abs(start.x - other.start.x) > eps())
            return start.x < other.start.x;
        if (std::abs(start.y - other.start.y) > eps())
            return start.y < other.start.y;
        if (std::abs(end.x - other.end.x) > eps())
            return end.x < other.end.x;
        return end.y < other.end.y;
    }

    constexpr inline bool IsVertical() const { return start.x == end.x; }
    constexpr inline double Length() const { return start.DistanceTo(end); }
    constexpr inline double Height() const { return std::max(start.y, end.y); }
    constexpr inline BoundingBox GetBoundingBox() const { return BoundingBox(start, end); }
    constexpr inline Point2D Center() const { return start + ((end - start) / 2.0); }
    constexpr inline Lines2D<2> Lines() const { return {{start.x, end.x}, {start.y, end.y}}; }
    constexpr inline Point2D Direction() const { return (end - start).Normalize(); }

    inline std::vector<Point2D> Vertices() const { return {start, end}; }

    // Коэффициенты прямой, которая содержит этот отрезок находим из уравнения:
    // y = k * x + b
    constexpr inline std::optional<std::pair<double, double>> LineCoeffs() const {
        if (std::abs(start.x - end.x) < eps()) {
            // Прямая типа x = Const
            return std::nullopt;
        } else {
            const double k = (start.y - end.y) / (start.x - end.x);
            const double b = start.y - k * start.x;
            return {{k, b}};
        }
    }

    // Возвращаем истину только если базовые прямые отрезков действительно параллельны.
    // Если два отрезка расположены на одной прямой, то возвращаем ложь
    constexpr inline bool IsParallelTo(const Line &other) const {
        const auto data = LineCoeffs();
        const auto other_data = other.LineCoeffs();

        if (data && other_data) {
            const auto [k, b] = *data;
            const auto [k_other, b_other] = *other_data;

            if (std::abs(k - k_other) < eps() && std::abs(b - b_other) >= eps()) {
                return true;
            } else {
                return false;
            }
        }
        else if (!data && !other_data && std::abs(start.x - other.start.x) >= eps()) {
            // Две прямые параллельны: x = C1, x = C2, C1 != C2
            return true;
        }
        else {
            return false;
        }
    }

    constexpr inline bool SharesSameLine(const Line &other) const {
        const auto data = LineCoeffs();
        const auto other_data = other.LineCoeffs();

        if (data && other_data) {
            const auto [k, b] = *data;
            const auto [k_other, b_other] = *other_data;

            if (std::abs(k - k_other) < eps() && std::abs(b - b_other) < eps()) {
                return true;
            } else {
                return false;
            }
        }
        else if (!data && !other_data && std::abs(start.x - other.start.x) < eps()) {
            // Две прямые тождественны: x = C1, x = C2, C1 == C2
            return true;
        }
        else {
            return false;
        }
    }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        auto len = start.DistanceTo(p) + end.DistanceTo(p);
        if (std::abs(len - Length()) < eps()) {
            return true;
        } else {
            return false;
        }
    }

    constexpr std::optional<Point2D> GetIntersectPoint(const Line &other) const {
        if (*this == other) {
            return { start + ((end - start) / 2.0) };
        }
        else if (IsParallelTo(other)) {
            return std::nullopt;
        }
        else if (SharesSameLine(other)) {
            std::vector<Point2D> points {start, end, other.start, other.end};

            if (IsVertical()) {
                if (end.y < other.start.y || other.end.y < start.y) {
                    return std::nullopt;
                }
                std::ranges::sort(points, {}, &Point2D::y);
            } else {
                if (end.x < other.start.x || other.end.x < start.x) {
                    return std::nullopt;
                }
                std::ranges::sort(points, {}, &Point2D::x);
            }

            return { Line{points[1], points[2]}.Center() };
        }
        else {
            const auto data = LineCoeffs();
            const auto other_data = other.LineCoeffs();

            if (data && other_data) {
                const auto [k, b] = *data;
                const auto [k_other, b_other] = *other_data;

                if (std::abs(b - b_other) < eps()) {
                    // Базовые прямые пересекаются в точке {0.0, b},
                    // Если эта точка принадлежит отрезкам, то они пересекаются
                    const auto p = Point2D{0.0, b};
                    if (ContainsPoint(p) && other.ContainsPoint(p)) {
                        return { std::move(p) };
                    } else {
                        return std::nullopt;
                    }
                } else {
                    const double x = (b_other - b) / (k - k_other);
                    const double y = k * x + b;
                    const auto p = Point2D{x, y};
                    if (ContainsPoint(p) && other.ContainsPoint(p)) {
                        return { std::move(p) };
                    } else {
                        return std::nullopt;
                    }
                }
            }
            else if (data && !other_data) {
                const auto [k, b] = *data;
                // Другая прямая представляет собой вертикальную линию x = С1,
                // Поэтому точка пересечения есть {C1, k * C1 + b}
                // Если эта точка принадлежит отрезкам, то они пересекаются
                const auto p = Point2D{other.start.x, k * other.start.x + b};
                if (ContainsPoint(p) && other.ContainsPoint(p)) {
                    return { std::move(p) };
                } else {
                    return std::nullopt;
                }
            }
            else if (!data && other_data) {
                const auto [k_other, b_other] = *other_data;
                // Первая прямая представляет собой вертикальную линию x = С1,
                // Поэтому точка пересечения есть {C1, k_other * C1 + b_other}
                // Если эта точка принадлежит отрезкам, то они пересекаются
                const auto p = Point2D{start.x, k_other * start.x + b_other};
                if (ContainsPoint(p) && other.ContainsPoint(p)) {
                    return { std::move(p) };
                } else {
                    return std::nullopt;
                }
            }
            else {
                // Здесь две вертикальные параллельные прямые, но этот случай обработан выше
                throw std::logic_error("Incorrect calculation of intersection point");
            }
        }
    }

    std::vector<Line> GetFaces() const {
        return { { start, end } };
    }

    constexpr inline bool Overlaps(const Line &other) const {
        const auto coeffs = LineCoeffs();
        const auto other_coeffs = other.LineCoeffs();

        if (coeffs && other_coeffs) {
            const auto [k, b] = *coeffs;
            const auto [k_other, b_other] = *other_coeffs;

            if (std::abs(k) < eps() && std::abs(k_other) < eps()) {
                // Прямые типа y = Const
                if (start.x <= other.start.x && other.start.x <= end.x) {
                    // Начало второго отрезка содержится в первом отрезке
                    return true;
                }
                else if (other.start.x <= start.x && start.x <= other.end.x) {
                    // Конец второго отрезка содержится в первом отрезке
                    return true;
                }
                else if (start.x <= other.start.x && other.end.x <= end.x) {
                    // Первый отрезок содержит второй
                    return true;
                }
                else if (other.start.x <= start.x && end.x <= other.end.x) {
                    // Второй отрезок содержит первый
                    return true;
                }
            }
            else if (std::abs(k) > eps() && std::abs(k_other) > eps() && std::abs(k - k_other) < 2.0 * eps()) {
                if (std::abs(b - b_other) < eps()) {
                    // Это одна и та же прямая - проверяем по координатам
                    if (start.x <= other.start.x && other.start.x <= end.x) {
                        // Начало второго отрезка содержится в первом отрезке
                        return true;
                    }
                    else if (other.start.x <= start.x && start.x <= other.end.x) {
                        // Конец второго отрезка содержится в первом отрезке
                        return true;
                    }
                    else if (start.x <= other.start.x && other.end.x <= end.x) {
                        // Первый отрезок содержит второй
                        return true;
                    }
                    else if (other.start.x <= start.x && end.x <= other.end.x) {
                        // Второй отрезок содержит первый
                        return true;
                    }
                } else {
                    // Это не равные друг другу параллельные прямые.
                    // Один из концов отрезков должен содержаться в другом отрезке
                    {
                        const double x = (start.x + k_other * (start.y - b_other)) / (k_other * k_other + 1.0);
                        const double y = k_other * x + b_other;
                        if (other.ContainsPoint({x, y})) {
                            return true;
                        }
                    }
                    {
                        const double x = (end.x + k_other * (end.y - b_other)) / (k_other * k_other + 1.0);
                        const double y = k_other * x + b_other;
                        if (other.ContainsPoint({x, y})) {
                            return true;
                        }
                    }
                    {
                        const double x = (other.start.x + k * (other.start.y - b)) / (k * k + 1.0);
                        const double y = k * x + b;
                        if (ContainsPoint({x, y})) {
                            return true;
                        }
                    }
                    {
                        const double x = (other.end.x + k * (other.end.y - b)) / (k * k + 1.0);
                        const double y = k * x + b;
                        if (ContainsPoint({x, y})) {
                            return true;
                        }
                    }
                }
            }
        }
        else if (!coeffs && !other_coeffs) {
            // Прямые типа x = Const
            if (start.y <= other.start.y && other.start.y <= end.y) {
                // Начало второго отрезка содержится в первом отрезке
                return true;
            }
            else if (other.start.y <= start.y && start.y <= other.end.y) {
                // Конец второго отрезка содержится в первом отрезке
                return true;
            }
            else if (start.y <= other.start.y && other.end.y <= end.y) {
                // Первый отрезок содержит второй
                return true;
            }
            else if (other.start.y <= start.y && end.y <= other.end.y) {
                // Второй отрезок содержит первый
                return true;
            }
        }
        return false;
    }

    Point2D GetRandomPoint() const {
        const auto get_random = [](const double l, const double r) -> double {
            std::random_device rd;
            std::default_random_engine re {rd()};
            // Здесь есть асимметрия, потому что левая граница включена, а правая нет,
            // но для простоты считаем это допустимым
            std::uniform_real_distribution<> dist(l, r);
            return dist(re);
        };
        if (std::abs(start.x - end.x) >= eps() && std::abs(start.y - end.y) >= eps()) {
            return Point2D{ get_random(start.x, end.x), get_random(start.y, end.y) };
        }
        else if (std::abs(start.x - end.x) < eps() && std::abs(start.y - end.y) >= eps()) {
            return Point2D{ start.x, get_random(start.y, end.y) };
        }
        else if (std::abs(start.x - end.x) >= eps() && std::abs(start.y - end.y) < eps()) {
            return Point2D{ get_random(start.x, end.x), start.y };
        }
        else {
            throw std::logic_error("Equal points can not form a line");
        }
        return {};
    }
};

struct Triangle {
    Point2D a, b, c;

    constexpr inline bool operator==(const Triangle &other) const = default;

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
    constexpr inline Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        const auto c1 = (b - a).Cross(p - a);
        const auto c2 = (c - b).Cross(p - b);
        const auto c3 = (a - c).Cross(p - c);
        // Если знаки одинаковые, то точка внутри треугольника
        if (c1 < 0.0 && c2 < 0.0 && c3 < 0.0) {
            return true;
        }
        else if (c1 > 0.0 && c2 > 0.0 && c3 > 0.0) {
            return true;
        }
        else if (std::abs(c1) < eps() || std::abs(c2) < eps() || std::abs(c3) < eps()) {
            // Если одно из произведений равно нулю, то точка лежит на границе треугольника
            return true;
        }
        else {
            return false;
        }
    }

    inline std::vector<Point2D> Vertices() const { return {a, b, c}; }
    inline std::vector<Line> GetFaces() const {
        return { { a, b }, { b, c }, { c, a } };
    }

    constexpr inline bool CircumCircleContainsPoint(const Point2D &p) const {
        return CircumCenter().DistanceTo(p) <= CircumRadius() + eps();
    }

    constexpr inline Point2D CircumCenter() const {
        const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

        if (std::abs(d) < eps()) {
            return { (a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0 };
        }

        const double ux = (
            (a.x * a.x + a.y * a.y) * (b.y - c.y) +
            (b.x * b.x + b.y * b.y) * (c.y - a.y) +
            (c.x * c.x + c.y * c.y) * (a.y - b.y)
        ) / d;

        const double uy = (
            (a.x * a.x + a.y * a.y) * (c.x - b.x) +
            (b.x * b.x + b.y * b.y) * (a.x - c.x) +
            (c.x * c.x + c.y * c.y) * (b.x - a.x)
        ) / d;

        return { ux, uy };
    }

    constexpr inline double CircumRadius() const {
        return CircumCenter().DistanceTo(a);
    }

    constexpr inline bool SharesFace(const Triangle &other) const {
        const std::span<const Point2D> this_points = {a, b, c};
        const std::span<const Point2D> other_points = {other.a, other.b, other.c};
        size_t shared_count = 0;

        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < eps() && std::abs(p1.y - p2.y) < eps()) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    constexpr inline bool SharesVertex(const Triangle &other) const {
        return
            a == other.a || a == other.b || a == other.c ||
            b == other.a || b == other.b || b == other.c ||
            c == other.a || c == other.b || c == other.c
        ;
    }
};

struct Rectangle {
    const Point2D left_bottom;
    double width, height;

    constexpr inline bool operator==(const Rectangle &other) const = default;

    constexpr inline double Area() const { return width * height; }
    constexpr inline Point2D Center() const { return left_bottom + Point2D{width/2.0, height/2.0}; }
    constexpr inline BoundingBox GetBoundingBox() const { return {left_bottom, left_bottom + Point2D{width, height}}; }
    constexpr inline double Height() const { return left_bottom.y + height; }
    constexpr inline double Width() const { return width; }
    constexpr inline Lines2D<1> Lines() const { return {{left_bottom.x}, {left_bottom.y}}; }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        const auto top_right = left_bottom + Point2D{width, height};
        return left_bottom <= p && p <= top_right;
    }

    inline std::vector<Point2D> Vertices() const {
        return {
            left_bottom,
            Point2D{left_bottom.x + width, left_bottom.y},
            Point2D{left_bottom.x, left_bottom.y + height},
            Point2D{left_bottom.x + width, left_bottom.y + height}
        };
    }
    inline std::vector<Line> GetFaces() const {
        return {
            { left_bottom, Point2D{left_bottom.x + width, left_bottom.y}  },
            { left_bottom, Point2D{left_bottom.x, left_bottom.y + height} },
            { Point2D{left_bottom.x + width, left_bottom.y},  Point2D{left_bottom.x + width, left_bottom.y + height} },
            { Point2D{left_bottom.x, left_bottom.y + height}, Point2D{left_bottom.x + width, left_bottom.y + height} }
        };
    }
};

struct RegularPolygon {
    const Point2D center_p;
    const double radius;
    const int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    constexpr inline bool operator==(const RegularPolygon &other) const = default;

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

    inline std::vector<Point2D> Vertices() const {
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

    inline Lines2DDyn Lines() const {
        auto vertices = Vertices();
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }

    constexpr inline double InnerRadius() const {
        return radius * std::cos(std::numbers::pi * sides);
    }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        // Разбиваем на треугольники и проверяем каждый из них
        const auto v = Vertices();
        for (int i = 0; i < v.size(); ++i) {
            const auto t = Triangle{ center_p, v[i], v[(i + 1) % v.size()] };
            if (t.ContainsPoint(p)) {
                return true;
            }
        }
        return false;
    }

    inline std::vector<Line> GetFaces() const {
        std::vector<Line> faces;
        const auto v = Vertices();
        for (int i = 0; i < v.size(); ++i) {
            faces.emplace_back(v[i], v[(i + 1) % v.size()]);
        }
        return faces;
    }
};

struct Circle {
    const Point2D center_p;
    const double radius;

    constexpr inline bool operator==(const Circle &other) const { return center_p == other.center_p && std::abs(radius - other.radius) < eps(); }

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    constexpr inline BoundingBox GetBoundingBox() const {
        return {
            { center_p.x - radius, center_p.y - radius },
            { center_p.x + radius, center_p.y + radius }
        };
    }

    constexpr inline double Height() const { return center_p.y + radius; }
    constexpr inline Point2D Center() const { return center_p; }

    inline std::vector<Point2D> Vertices(size_t N = 30) const {
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

    inline Lines2DDyn Lines(size_t N = 100) const {
        auto vertices = Vertices(N);
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }

    Point2D GetRandomPoint() const {
        const auto get_random = [](const double l, const double r) -> double {
            std::random_device rd;
            std::default_random_engine re {rd()};
            // Здесь есть асимметрия, потому что левая граница включена, а правая нет,
            // но для простоты считаем это допустимым
            std::uniform_real_distribution<> dist(l, r);
            return dist(re);
        };
        return {
            get_random(center_p.x - radius, center_p.x + radius),
            get_random(center_p.y - radius, center_p.y + radius)
        };
    }

    constexpr inline bool DoNotIntersectCircle(const Circle &other) const {
        auto dist = Center().DistanceTo(other.Center());

        if (dist > radius + other.radius) {
            return true; // Слишком далеко друг от друга
        }
        else if (dist < std::abs(radius - other.radius)) {
            return true; // Малый круг внутри большого
        }
        else {
            // Или касаются, или пересекаются
            return false;
        }
    }

    constexpr std::optional<Point2D> GetIntersectPoint(const Circle &other) const {
        if (*this == other) {
            return GetRandomPoint();
        }
        else if (DoNotIntersectCircle(other)) {
            return std::nullopt;
        }
        else {
            /*
                Перенесём систему координат в центр текущей окружности.
                И будем от неё плясать, координаты центра будут {0, 0}.
            */
            auto new_other_center = other.center_p - center_p;
            const double a = new_other_center.Dot(new_other_center);
            const double c = (radius*radius + a - other.radius*other.radius) / 2.0;
            const double b = -2.0 * new_other_center.y * c;
            const double e = c*c - radius*radius*new_other_center.x*new_other_center.x;

            if (std::abs(new_other_center.x) < eps()) {
                const double y = c / new_other_center.y;
                const double D = radius*radius - y*y;
                if (std::abs(D) < eps()) {
                    // Окружности касаются друг друга
                    const double x = 0.0;
                    return {{ x + new_other_center.x, y + new_other_center.y }};
                }
                else if (D > 0) {
                    // Пересекаются в двух точках. Берём одну из них.
                    const double x = std::sqrt(D);
                    return {{ x + new_other_center.x, y + new_other_center.y }};
                }
                else {
                    throw std::logic_error("Circles do not touch each other!");
                }
            } else {
                const double D = b*b - 4.0*a*e;
                if (std::abs(D) < eps()) {
                    // Окружности касаются друг друга
                    const double y = -b / (2.0 * a);
                    const double x = (c - y * new_other_center.y) / new_other_center.x;
                    return {{ x + new_other_center.x, y + new_other_center.y }};
                }
                else if (D > 0) {
                    const double y = (-b + std::sqrt(D)) / (2.0 * a);
                    const double x = (c - y * new_other_center.y) / new_other_center.x;
                    return {{ x + new_other_center.x, y + new_other_center.y }};
                }
                else {
                    throw std::logic_error("Circles do not intersect each other!");
                }
            }
        }
    }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        return (p.x - center_p.x) * (p.x - center_p.x) + (p.y - center_p.y) * (p.y - center_p.y) <= radius * radius;
    }

    std::vector<Line> GetFaces() const {
        throw std::logic_error("Circle has no faces");
        return {};
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

    constexpr inline bool operator==(const Polygon &other) const = default;

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

    inline std::vector<Point2D> Vertices() const { return vertices; }

    inline Lines2DDyn Lines() const {
        auto vertices = Vertices();
        auto res = Lines2DDyn{};
        res.Reserve(vertices.size() + 1);
        for (const auto &v : vertices) {
            res.PushBack(v);
        }
        res.PushBack(res.Front());
        return res;
    }

    constexpr inline bool ContainsPoint(const Point2D &p) const {
        // Алгоритм вероятностный. Простого ответа на вопрос принадлежит ли точка
        // многоугольнику нет, т.к. многоугольник может быть какой угодно "сложный".
        // Здесь используется метод трассировки луча. Если точка расположена внутри,
        // то "с большой вероятностью" число пересечений луча из этой точки в каком-то
        // произвольном направлении будет нечётным. Если снаружи, то число пересечений луча
        // с гранями многоугольника "с большой вероятностью" будет чётным.
        // Чтобы уменьшить вероятность неверного ответа, делаем N рандомных попыток и
        // смотрим, каких результатов получилось больше: чётных или нечётных
        constexpr size_t N = 3; // нечётное, чтобы можно было определить победителя
        std::vector<int> res(N);

        for (size_t i = 0; i < N; ++i) {
            const int num_intersections = calculateIntersectionsWithRandomRay(p);
            res[i] = num_intersections % 2 ? 1 : -1;
        }

        return std::ranges::fold_left(res, 0, std::plus<int>()) < 0;
    }

    inline std::vector<Line> GetFaces() const {
        std::vector<Line> faces;
        const auto v = Vertices();
        for (int i = 0; i < v.size(); ++i) {
            faces.emplace_back(v[i], v[(i + 1) % v.size()]);
        }
        return faces;
    }

private:
    constexpr inline size_t calculateIntersectionsWithRandomRay(const Point2D &p) const {
        // Случайным образом выбираем одну из граней многоугольника и проводим луч
        // из точки `p` через рандомную точку этой грани за границу BoundingBox этого многоугольника.
        // Далее считаем количество пересечений этого луча (по факту это отрезок) с гранями
        // многоугольника
        const auto faces = GetFaces();
        const auto get_random = [](const int sz) -> int {
            std::random_device rd;
            std::default_random_engine re {rd()};
            std::uniform_int_distribution<int> dist(0, sz);
            return dist(re);
        };
        // Эта точка нужна для формирования луча
        const auto random_point = faces[get_random(faces.size() - 1)].GetRandomPoint();
        const auto coeffs = Line{p, random_point}.LineCoeffs();

        double boundary_x, boundary_y;

        if (coeffs) {
            const auto [k, b] = *coeffs;

            if (p.x < random_point.x) {
                boundary_x = GetBoundingBox().Right();
            } else {
                boundary_x = GetBoundingBox().Left();
            }

            boundary_y = k * boundary_x + b;
        }
        else {
            // Получилась вертикальная прямая x = Const
            boundary_x = p.x;

            if (p.y > random_point.y) {
                // Луч идёт вниз, значит в качестве крайней точки берём минимальную координату
                // ограничивающего прямоугольника
                boundary_y = GetBoundingBox().Bottom();
            } else {
                // Луч идёт вверх
                boundary_y = GetBoundingBox().Top();
            }
        }

        const auto ray = Line{p, Point2D{boundary_x, boundary_y}};
        size_t res = 0;

        for (const auto face : faces) {
            if (face.GetIntersectPoint(ray)) {
                ++res;
            }
        }

        return res;
    }
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenerateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

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
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.left_bottom, r.width, r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) const { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius, p.sides);
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
