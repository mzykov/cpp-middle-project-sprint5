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
    
}

TEST(TestGeometry, TestPointToRegularPolygonDistance) {
    
}

TEST(TestGeometry, TestPointToCircleDistance) {
    
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
