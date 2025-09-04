#include <gtest/gtest.h>

#include "queries.hpp"

using namespace geometry;

TEST(TestGeometry, TestPointToLineDistance) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Shape
        line0 = Line{origin, {1.2, 3.4}},
        line1 = Line{{0.0, std::sqrt(2.0)}, {std::sqrt(2.0), 0.0}},
        line2 = Line{{5.0, 17.0}, {6.0, 20.0}},      // y = 3*x + 2
        x_const1 = Line{{1.0, -3.0}, {1.0, 20.0}},   // x = 1.0
        x_const2 = Line{{2.0, -3.0}, {2.0, -1.5}},   // x = 2.0
        y_const1 = Line{{-77.0, -1.0}, {3.4, -1.0}}, // y = -1.0
        y_const2 = Line{{0.0, -2.0}, {5.0, -2.0}}    // y = -2.0
    ;
    // when
    // then
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(line0, origin), 0.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(line1, origin), 1.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(line2, origin), std::sqrt(5.0*5.0 + 17.0*17.0));
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(x_const1, origin), 1.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(x_const2, origin), std::sqrt(6.25));
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(y_const1, origin), 1.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(y_const2, origin), 2.0);
}

TEST(TestGeometry, TestPointToTriangleDistance) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Shape
        tri0 = Triangle{origin, {1.3, 3.4}, {1.4, -2.7}},
        tri1 = Triangle{{-10.5, 0.1}, {2.3, 13.4}, {2.4, -22.7}},
        tri2 = Triangle{{5.3, 0.0}, {12.888, 113.4}, {208.4, -0.7}},
        tri3 = Triangle{{0.0, std::sqrt(2.0)}, {std::sqrt(2.0), 0.0}, {1e8, 1e8}}
    ;
    // when
    // then
    for (const auto &shape : {tri0, tri1, tri2}) {
        const auto vertices = shape.visit([](auto &&tri){ return tri.Vertices(); });
        for (const auto &v : vertices) {
            EXPECT_DOUBLE_EQ(queries::DistanceToPoint(shape, v), 0.0);
        }
    }
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(tri0, origin), 0.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(tri1, origin), 0.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(tri2, origin), 5.3);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(tri3, origin), 1.0);
}

TEST(TestGeometry, TestPointToRectangleDistance) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Shape
        square = Rectangle{{-1.0, -1.0}, 2.0, 2.0},
        rect0  = Rectangle{{3.77, -44.8888}, 99.1, 42.42}
    ;
    // when
    // then
    for (const auto &shape : {square, rect0}) {
        const auto vertices = shape.visit([](auto &&rect){ return rect.Vertices(); });
        for (const auto &v : vertices) {
            EXPECT_DOUBLE_EQ(queries::DistanceToPoint(shape, v), 0.0);
        }
    }
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(square, origin), 0.0);
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(square, Point2D{1.1, 0.0}) - 0.1)     < eps()));
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(square, Point2D{0.0, 1.1}) - 0.1)     < eps()));
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(square, Point2D{1.1, 1.0}) - 0.1)     < eps()));
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(square, Point2D{1.0, 1.1}) - 0.1)     < eps()));
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(rect0, origin) - 4.50642579435188)    < eps()));
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(rect0, Point2D{3.2, -22.333}) - 0.57) < eps()));
}

TEST(TestGeometry, TestPointToRegularPolygonDistance) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Shape
        triangle = RegularPolygon{origin, 5.0, 3},
        square   = RegularPolygon{origin, 1.0, 4},
        pentagon = RegularPolygon{origin, 33.3333333333, 5},
        hexagon  = RegularPolygon{Point2D{1.0, 3.0}, 2.71, 6},
        heptagon = RegularPolygon{origin, 0.0005, 7},
        octagon  = RegularPolygon{Point2D{-5.0, -3.0}, 1e5, 8}
    ;
    const auto get_random_point_out_of_shape_on_line = [](const Line &l, const Point2D &c) -> Point2D {
        const auto get_random_coord = [](const double l, const double r) -> double {
            std::random_device rd;
            std::default_random_engine re {rd()};
            std::uniform_real_distribution<> dist(l, r);
            return dist(re);
        };
        if (l.IsVertical()) {
            double lo, hi;
            if (l.end == c) {
                lo = l.start.y - 2.0;
                hi = l.start.y - 1.0;
            } else {
                lo = l.end.y + 1.0;
                hi = l.end.y + 2.0;
            }
            return { c.x, get_random_coord(lo, hi) };
        }
        else {
            double lo, hi;
            if (l.end == c) {
                lo = l.start.x - 2.0;
                hi = l.start.x - 1.0;
            } else {
                lo = l.end.x + 1.0;
                hi = l.end.x + 2.0;
            }
            const auto [k, b] = *l.LineCoeffs();
            const double x = get_random_coord(lo, hi);
            return { x,  k*x + b };
        }
    };
    // when
    // then
    for (const auto &shape : {triangle, square, pentagon, hexagon, heptagon, octagon}) {
        const auto center   = shape.visit([](auto &&poly){ return poly.Center(); });
        const auto vertices = shape.visit([](auto &&poly){ return poly.Vertices(); });
        const auto faces    = shape.visit([](auto &&poly){ return poly.GetFaces(); });

        EXPECT_DOUBLE_EQ(queries::DistanceToPoint(shape, center), 0.0);

        for (const auto &v : vertices) {
            EXPECT_DOUBLE_EQ(queries::DistanceToPoint(shape, v), 0.0);
            const auto p = get_random_point_out_of_shape_on_line(Line{center, v}, center);
            EXPECT_TRUE((std::abs(queries::DistanceToPoint(shape, p) - Line{p, v}.Length()) < eps()));
        }

        for (const auto &face : faces) {
            const auto p = get_random_point_out_of_shape_on_line(Line{center, face.Center()}, center);
            EXPECT_TRUE((std::abs(queries::DistanceToPoint(shape, p) - queries::DistanceToPoint(Shape{face}, p)) < eps()));
        }
    }
}

TEST(TestGeometry, TestPointToCircleDistance) {
    // given
    constexpr Point2D origin{0.0, 0.0}, egypt{3.0, 4.0};
    constexpr Shape
        circle0 = Circle{origin, 1.0},
        circle1 = Circle{{-7.1, 2.7}, 3.5}
    ;
    // when
    // then
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle0, egypt), 4.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle0, Point2D{0.1, 0.1}), 0.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle0, Point2D{0.0, 1.0}), 0.0);
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle0, Point2D{1.0, 0.0}), 0.0);
    EXPECT_TRUE((std::abs(queries::DistanceToPoint(circle1, egypt) - 6.68331969448077) < eps()));
}

TEST(TestGeometry, TestPointToPolygonDistance) {
    
}

TEST(TestGeometry, TestDistanceBetweenLines) {
    
}

TEST(TestGeometry, TestDistanceBetweenCircles) {
    // given
    constexpr Shape
        circle0 = Circle{{0.0, 0.0}, 1.0},
        circle1 = Circle{{-3.0, 4.0}, 3.99},
        circle2 = Circle{{5.3, 0.0}, 5.0},
        circle3 = Circle{{0.1, 0.1}, 0.1}
    ;

    // when
    const auto got01 = queries::DistanceBetweenShapes(circle0, circle1);
    // then
    EXPECT_TRUE(got01.has_value());
    EXPECT_TRUE((std::abs(got01.value() - 0.01) < eps()));

    // when
    const auto got02 = queries::DistanceBetweenShapes(circle0, circle2);
    // then
    EXPECT_TRUE(got02.has_value());
    EXPECT_DOUBLE_EQ(got02.value(), 0.0);

    // when
    const auto got03 = queries::DistanceBetweenShapes(circle0, circle3);
    // then
    EXPECT_TRUE(got03.has_value());
    EXPECT_DOUBLE_EQ(got03.value(), 0.0);

    // when
    const auto got12 = queries::DistanceBetweenShapes(circle1, circle2);
    // then
    EXPECT_TRUE(got12.has_value());
    EXPECT_TRUE((std::abs(got12.value() - 0.223576938409968) < eps()));

    // when
    const auto got13 = queries::DistanceBetweenShapes(circle1, circle3);
    // then
    EXPECT_TRUE(got13.has_value());
    EXPECT_TRUE((std::abs(got13.value() - 0.89196748283246) < eps()));
}
