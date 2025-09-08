#include <gtest/gtest.h>

#include "geometry.hpp"

using namespace geometry;

TEST(TestGeometry, TestPoint) {
    // given
    constexpr Point2D origin{0.0, 0.0}, point0;
    constexpr Point2D point1{1.0, 1.0}, point2{2.0, 2.0}, point3{3.0, 4.0};
    // when
    // then
    EXPECT_TRUE(point0 == origin);
    EXPECT_TRUE((point1 + (Point2D{eps(), eps()} * 2.0)) != point1);
    EXPECT_TRUE(point1 < point2);
    EXPECT_TRUE(point2 <= point2);
    EXPECT_TRUE((point1 + point1) == point2);
    EXPECT_TRUE((point2 - point1) == point1);
    EXPECT_TRUE((point1 * 2.0) == point2);
    EXPECT_TRUE(point2/2.0 == point1);
    EXPECT_DOUBLE_EQ(point2.Dot(point2), 8.0);
    EXPECT_DOUBLE_EQ((Point2D{1.0, 0.0}.Cross(point1)), 1.0);
    EXPECT_DOUBLE_EQ(point3.Length(), 5.0);
    EXPECT_DOUBLE_EQ(point2.DistanceTo(point1), std::sqrt(2.0));
    EXPECT_TRUE((point3.Normalize()) == (Point2D{0.6, 0.8}));
}

TEST(TestGeometry, TestBoundingBox) {
    // given
    constexpr Point2D origin{0.0, 0.0}, bbox1_center{1.0, 0.5};
    constexpr BoundingBox
        bbox1{{-2.0, -3.0}, {4.0, 4.0}},
        bbox1_inv{{4.0, 4.0}, {-2.0, -3.0}},
        bbox2{{-1.1, -1.5}, {0.3, 0.9}},
        bbox3{{-3.0, -4.0}, origin}
    ;
    // when
    // then
    EXPECT_TRUE(bbox1 == bbox1_inv);
    EXPECT_TRUE(bbox1 != bbox2);
    EXPECT_TRUE(bbox1.Overlaps(bbox2));
    EXPECT_TRUE(bbox1.Overlaps(bbox3));
    EXPECT_TRUE(bbox1.Contains(bbox2));
    EXPECT_TRUE(bbox1.ContainsPoint(origin));
    EXPECT_TRUE(bbox3.ContainsPoint(origin));
    EXPECT_DOUBLE_EQ(bbox1.Width(), 6.0);
    EXPECT_DOUBLE_EQ(bbox1.Height(), 7.0);
    EXPECT_TRUE((bbox1.Center()) == bbox1_center);
}

TEST(TestGeometry, TestLine) {
    // given
    constexpr Point2D
        origin{0.0, 0.0},
        point1{3.0, 4.0},
        point2{4.57777777, 4.57777777},
        center1{1.5, 2.0},
        center2{2.5, 3.0}
    ;
    constexpr Line
        line1{origin, point1},
        line1_inv{point1, origin},
        line1_par{{-1.0, -1.0}, {2.0, 3.0}},
        line2{{2.0, 2.0}, point1},
        line45_1{{1.0, 1.0}, {2.0, 2.0}},
        line45_15{{1.5, 1.5}, {2.5, 2.5}},
        line45_2{{3.0, 3.0}, {5.0, 5.0}},
        line45_3{{-1.0, -1.0}, {10.0, 10.0}},
        x_const1{{-3.51, 7.1}, {-3.51, 7.2}},
        x_const1_u{{-3.51, 70.1}, {-3.51, 74.002}},
        x_const2{{0.00003, -1.0}, {0.00003, 3.7}},
        x_const3{{1.0, 1.0}, {1.0, 2.0}},
        x_const3_o{{1.0, 0.5}, {1.0, 2.5}},
        y_const1{{5.2, -0.03}, {-0.99, -0.03}},
        y_const1_r{{1.2, -0.03}, {1e8, -0.03}},
        y_const2{{1.1, 1e7}, {1.11, 1e7}}
    ;
    // when
    // then
    EXPECT_TRUE(line1 == line1_inv);
    EXPECT_TRUE(line1 != line2);
    EXPECT_TRUE(line1 < line2);
    EXPECT_DOUBLE_EQ(line1.Length(), 5.0);
    EXPECT_DOUBLE_EQ(line2.Height(), 4.0);
    EXPECT_TRUE((line1.GetBoundingBox() == BoundingBox{origin, point1}));
    EXPECT_TRUE((line1.Center() == center1));
    EXPECT_TRUE((line2.Center() == center2));
    EXPECT_FALSE(line1.IsParallelTo(line2));
    EXPECT_TRUE(line1.IsParallelTo(line1_par));
    EXPECT_TRUE(x_const1.IsParallelTo(x_const2));
    EXPECT_TRUE(y_const1.IsParallelTo(y_const2));
    EXPECT_FALSE(line1.SharesSameLine(line2));
    EXPECT_TRUE(line45_1.SharesSameLine(line45_2));
    EXPECT_TRUE(x_const1.SharesSameLine(x_const1_u));
    EXPECT_TRUE(y_const1.SharesSameLine(y_const1_r));
    EXPECT_FALSE(line1.SharesSameLine(line1_par));
    EXPECT_FALSE(x_const1.SharesSameLine(x_const2));
    EXPECT_FALSE(y_const1.SharesSameLine(y_const2));
    EXPECT_FALSE(line1.SharesSameLine(y_const2));
    EXPECT_FALSE(y_const1.SharesSameLine(line2));
    EXPECT_FALSE(line45_2.ContainsPoint(point1));
    EXPECT_TRUE(line45_2.ContainsPoint(point2));
    EXPECT_FALSE(line45_1.ContainsPoint(point1));
    EXPECT_FALSE(line45_1.ContainsPoint(point2));
    EXPECT_TRUE((line1.GetIntersectPoint(line1_inv).value() == center1));
    EXPECT_FALSE(line1.GetIntersectPoint(line1_par));
    EXPECT_FALSE(line2.GetIntersectPoint(line1_par));
    EXPECT_FALSE(line45_1.GetIntersectPoint(line45_2));
    EXPECT_FALSE(x_const1.GetIntersectPoint(x_const1_u));
    EXPECT_TRUE((line45_1.GetIntersectPoint(line45_15).value() == Point2D{1.75, 1.75}));
    EXPECT_TRUE((line45_1.GetIntersectPoint(line45_3).value() == Point2D{1.5, 1.5}));
    EXPECT_TRUE((x_const3.GetIntersectPoint(x_const3_o).value() == Point2D{1.0, 1.5}));
    EXPECT_TRUE((line1.GetIntersectPoint(line2).value() == point1));
    EXPECT_TRUE(x_const2.GetIntersectPoint(line1_par).has_value());
    EXPECT_TRUE(line1_par.GetIntersectPoint(x_const2).has_value());
    EXPECT_TRUE(line1_par.GetIntersectPoint(y_const1).has_value());
    EXPECT_TRUE(y_const1.GetIntersectPoint(line1_par).has_value());
    EXPECT_TRUE(line1.Overlaps(line1));
    EXPECT_FALSE(line45_1.Overlaps(line45_2));
    EXPECT_TRUE(line45_1.Overlaps(line45_15));
    EXPECT_FALSE(line45_2.Overlaps(line45_15));
    EXPECT_TRUE(line45_3.Overlaps(line45_1));
    EXPECT_TRUE(line45_3.Overlaps(line45_15));
    EXPECT_TRUE(line45_3.Overlaps(line45_2));
    EXPECT_FALSE(x_const1.Overlaps(x_const1_u));
    EXPECT_TRUE(x_const1.Overlaps(x_const1));
    EXPECT_TRUE(x_const3.Overlaps(x_const3_o));
    EXPECT_TRUE(y_const1.Overlaps(y_const1_r));
    EXPECT_FALSE(y_const2.Overlaps(y_const1_r));
    EXPECT_TRUE(line1.ContainsPoint(line1.GetRandomPoint()));
    EXPECT_FALSE(line1.ContainsPoint(line1_par.GetRandomPoint()));
    EXPECT_TRUE(line2.ContainsPoint(line2.GetRandomPoint()));
    EXPECT_TRUE(x_const1.ContainsPoint(x_const1.GetRandomPoint()));
    EXPECT_TRUE(y_const1.ContainsPoint(y_const1.GetRandomPoint()));
}

TEST(TestGeometry, TestTriangle) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Triangle
        egypt{{1.0, 1.0}, {1.0, 4.0}, {5.0, 1.0}},
        egypt_mirrorx{{1.0, 1.0}, {1.0, -2.0}, {5.0, 1.0}},
        arbitrary_triangle{{1.0, 1.0}, {-13.8, 5.9997}, {100.44, -600.77}}
    ;
    // when
    // then
    EXPECT_TRUE((egypt == egypt));
    EXPECT_DOUBLE_EQ(egypt.Area(), 6.0);
    EXPECT_DOUBLE_EQ(egypt.Height(), 4.0);
    EXPECT_TRUE((egypt.Center() == Point2D{7.0/3.0, 2.0}));
    EXPECT_TRUE((egypt.GetBoundingBox() == BoundingBox{{1.0, 1.0}, {5.0, 4.0}}));
    EXPECT_TRUE(egypt.ContainsPoint(egypt.Center()));
    EXPECT_FALSE(egypt.ContainsPoint(Point2D{1e8, 1e9}));
    EXPECT_FALSE(egypt.ContainsPoint(origin));
    EXPECT_FALSE(egypt_mirrorx.ContainsPoint(origin));
    EXPECT_TRUE(arbitrary_triangle.ContainsPoint(origin));

    for (const auto &v : egypt.Vertices()) {
        EXPECT_TRUE(egypt.ContainsPoint(v));
        EXPECT_TRUE(egypt.CircumCircleContainsPoint(v));
    }

    EXPECT_TRUE((egypt.CircumCenter() == Point2D{3.0, 2.5}));
    EXPECT_FALSE(egypt.CircumCircleContainsPoint(origin));
    EXPECT_TRUE(egypt.CircumCircleContainsPoint(Point2D{3.1, 2.6}));
    EXPECT_TRUE(egypt.CircumCircleContainsPoint(Point2D{5.0, 4.0}));
    EXPECT_TRUE(egypt.CircumCircleContainsPoint(egypt.Center()));
    EXPECT_TRUE(egypt.SharesFace(egypt_mirrorx));
    EXPECT_TRUE(egypt.SharesVertex(egypt_mirrorx));
    EXPECT_TRUE(egypt.SharesVertex(arbitrary_triangle));
    EXPECT_TRUE(arbitrary_triangle.SharesVertex(egypt_mirrorx));
}

TEST(TestGeometry, TestRectangle) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Rectangle
        square{origin, 1.0, 1.0},
        arbitrary_rectangle{Point2D{-5.0, -3.0}, 11.0, 11.0}
    ;
    // when
    // then
    EXPECT_TRUE((square == square));
    EXPECT_TRUE((square != arbitrary_rectangle));
    EXPECT_DOUBLE_EQ(square.Area(), 1.0);
    EXPECT_DOUBLE_EQ(arbitrary_rectangle.Area(), 121.0);
    EXPECT_TRUE((square.Center() == Point2D{0.5, 0.5}));
    EXPECT_TRUE((arbitrary_rectangle.Center() == Point2D{0.5, 2.5}));
    EXPECT_TRUE((square.GetBoundingBox() == BoundingBox{origin, {1.0, 1.0}}));
    EXPECT_TRUE((arbitrary_rectangle.GetBoundingBox() == BoundingBox{arbitrary_rectangle.left_bottom, {6.0, 8.0}}));
    EXPECT_TRUE(square.ContainsPoint(square.Center()));
    EXPECT_TRUE(arbitrary_rectangle.ContainsPoint(arbitrary_rectangle.Center()));
    EXPECT_TRUE(arbitrary_rectangle.ContainsPoint(square.Center()));

    for (const auto &v : square.Vertices()) {
        EXPECT_TRUE(square.ContainsPoint(v));
        EXPECT_TRUE(arbitrary_rectangle.ContainsPoint(v));
    }
    for (const auto &v : arbitrary_rectangle.Vertices()) {
        EXPECT_TRUE(arbitrary_rectangle.ContainsPoint(v));
        EXPECT_FALSE(square.ContainsPoint(v));
    }
}

TEST(TestGeometry, TestRegularPolygon) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr RegularPolygon
        triangle{origin, 5.0, 3},
        square{origin, 1.0, 4},
        pentagon{origin, 33.3333333333, 5},
        hexagon{Point2D{1.0, 3.0}, 2.71, 6},
        heptagon{origin, 0.0005, 7},
        octagon{Point2D{-5.0, -3.0}, 1e5, 8}
    ;
    // when
    // then
    EXPECT_TRUE((triangle == triangle));
    EXPECT_TRUE((square == square));
    EXPECT_TRUE((pentagon != hexagon));
    EXPECT_TRUE((heptagon != octagon));
    EXPECT_TRUE((std::abs(triangle.Area() - 32.47595264191650)    < eps()));
    EXPECT_TRUE((std::abs(square.Area()   - 2.000000000000000)    < eps()));
    EXPECT_TRUE((std::abs(pentagon.Area() - 2641.823656370143)    < eps()));
    EXPECT_TRUE((std::abs(hexagon.Area()  - 19.08053150379988)    < eps()));
    EXPECT_TRUE((std::abs(heptagon.Area() - 6.84102547159526e-07) < eps()));
    EXPECT_TRUE((std::abs(octagon.Area()  - 28284271247.4619)     < eps()));

    for (const auto &figure : {triangle, square, pentagon, hexagon, heptagon, octagon}) {
        EXPECT_DOUBLE_EQ(figure.GetBoundingBox().Top(), figure.Height());
        EXPECT_TRUE(figure.ContainsPoint(figure.Center()));

        const auto get_random_point = [](const RegularPolygon &f) -> Point2D {
            std::random_device rd;
            std::default_random_engine re {rd()};
            std::uniform_real_distribution<> dist_r(0.0, (1.0 - eps()) * f.InnerRadius());
            std::uniform_real_distribution<> dist_phi(0.0, 2.0 * std::numbers::pi);
            const double r = dist_r(re);
            const double phi = dist_phi(re);
            return f.Center() + (Point2D{std::cos(phi), std::sin(phi)} * r);
        };
        EXPECT_TRUE(figure.ContainsPoint(get_random_point(figure)));

        for (const auto &v : figure.Vertices()) {
            EXPECT_TRUE(figure.ContainsPoint(v));
        }
    }
}

TEST(TestGeometry, TestCircle) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Circle
        BigO{origin, 1.0},
        RightO{Point2D{2.0, 0.0}, 1.0},
        TopO{Point2D{0.0, 2.0}, 1.0},
        LeftO{Point2D{-1.0, 0.0}, 1.1},
        ShiftedO{Point2D{-3.4555, 17.888}, 44.777777}
    ;
    // when
    // then
    EXPECT_TRUE((BigO == BigO));
    EXPECT_TRUE((BigO != ShiftedO));
    EXPECT_DOUBLE_EQ(BigO.GetBoundingBox().Area(), 4.0);
    EXPECT_DOUBLE_EQ(BigO.Height(), 1.0);
    EXPECT_FALSE(BigO.GetIntersectPoint(ShiftedO).has_value());
    EXPECT_FALSE(ShiftedO.GetIntersectPoint(BigO).has_value());
    EXPECT_TRUE((BigO.GetIntersectPoint(RightO).value() == Point2D{1.0, 0.0}));
    EXPECT_TRUE((TopO.GetIntersectPoint(BigO).value() == Point2D{0.0, 1.0}));
    EXPECT_TRUE(LeftO.GetIntersectPoint(BigO).has_value());

    for (const auto &circle : {BigO, RightO, TopO, LeftO, ShiftedO}) {
        EXPECT_DOUBLE_EQ(circle.GetBoundingBox().Top(), circle.Height());
        EXPECT_TRUE(circle.ContainsPoint(circle.Center()));

        const auto get_random_point = [](const Circle &c) -> Point2D {
            std::random_device rd;
            std::default_random_engine re {rd()};
            std::uniform_real_distribution<> dist_r(0.0, (1.0 - eps()) * c.radius);
            std::uniform_real_distribution<> dist_phi(0.0, 2.0 * std::numbers::pi);
            const double r = dist_r(re);
            const double phi = dist_phi(re);
            return c.Center() + (Point2D{std::cos(phi), std::sin(phi)} * r);
        };
        EXPECT_TRUE(circle.ContainsPoint(get_random_point(circle)));
    }
}

TEST(TestGeometry, TestPolygon) {
    // given
    constexpr Point2D origin{0.0, 0.0}, out_of_flag{3.222222, 4.17};
    constexpr RegularPolygon
        triangle{origin, 5.0, 3},
        square{origin, 1.0, 4},
        pentagon{origin, 33.3333333333, 5},
        hexagon{Point2D{0.1, -0.2}, 2.71, 6},
        heptagon{origin, 0.0005, 7},
        octagon{Point2D{-5.0, -3.0}, 1e5, 8}
    ;
    const Polygon
        looks_like_flag{{{0.0, 0.0}, {11.1, 0.0}, {std::numbers::pi, 4.17}, {10.9, 8.2111}, {0.0, 8.2111}}},
        convex{{{0.0, 0.0}, {5.0, 0.0}, {8.0, 1.0}, {9.0, 3.0}, {8.0, 5.0}, {5.0, 5.0}, {1.0, 3.0}}}
    ;
    const std::vector<Point2D> out_of_convex {
        {0.0, -1.1}, {8.0, 0.0}, {9.0, 1.0}, {8.0, 7.0}, {5.0, 6.5}, {0.0, 3.0}, {-1.0, -1.0}
    };

    // when
    // then
    EXPECT_FALSE(looks_like_flag.ContainsPoint(out_of_flag));
    EXPECT_FALSE(looks_like_flag.ContainsPoint(looks_like_flag.Center()));

    for (const auto &v : looks_like_flag.Vertices()) {
        EXPECT_TRUE(looks_like_flag.ContainsPoint(v));
    }
    for (const auto &v : convex.Vertices()) {
        EXPECT_TRUE(convex.ContainsPoint(v));
    }
    for (const auto &p : out_of_convex) {
        EXPECT_FALSE(convex.ContainsPoint(p));
    }
    for (const auto &regular_polygon : {triangle, square, pentagon, hexagon, heptagon, octagon}) {
        const Polygon arbitrary_polygon{regular_polygon.Vertices()};
        EXPECT_TRUE((arbitrary_polygon.Center() == regular_polygon.Center()));
        EXPECT_TRUE((arbitrary_polygon.Height() == regular_polygon.Height()));
        EXPECT_TRUE((arbitrary_polygon.GetBoundingBox() == regular_polygon.GetBoundingBox()));

        for (const auto &v : arbitrary_polygon.Vertices()) {
            EXPECT_TRUE(arbitrary_polygon.ContainsPoint(v));
        }
    }
}
