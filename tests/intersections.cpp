#include <gtest/gtest.h>

#include "intersections.hpp"

using namespace geometry;

TEST(TestIntersections, TestLineIntersectsCircleAtOnePoint) {
    // given
    constexpr Point2D
        origin{0.0, 0.0},
        x30degree{std::cos(std::numbers::pi/6.0), std::sin(std::numbers::pi/6.0)}
    ;
    constexpr Shape
        line   = Line{origin, x30degree * 10.0},
        circle = Circle{origin, 1.0}
    ;
    // when
    const auto got = intersections::GetIntersectPoint(line, circle);
    // then
    EXPECT_TRUE(got.has_value());              // check expected
    EXPECT_TRUE(got.value().has_value());      // check optional
    EXPECT_EQ(got.value().value(), x30degree); // check point
}

TEST(TestIntersections, TestLineIntersectsCircleAtTwoPoints) {
    // given
    constexpr Point2D
        origin{0.0, 0.0},
        x30degree{std::cos(std::numbers::pi/6.0), std::sin(std::numbers::pi/6.0)}
    ;
    constexpr Shape
        line   = Line{x30degree * (-10.0), x30degree * 10.0},
        circle = Circle{origin, 1.0}
    ;
    // when
    const auto got = intersections::GetIntersectPoint(line, circle);
    // then
    EXPECT_TRUE(got.has_value());         // check expected
    EXPECT_TRUE(got.value().has_value()); // check optional
    EXPECT_TRUE((got.value().value() == x30degree || got.value().value() == x30degree * (-1.0))); // check points
}

TEST(TestIntersections, TestCircleTouchesLines) {
    // given
    constexpr Point2D
        origin{0.0, 0.0},
        x45degree{1.0/std::sqrt(2.0), 1.0/std::sqrt(2.0)}
    ;
    constexpr Shape
        line_in      = Line{origin, x45degree},
        line_out     = Line{x45degree, {77.77, 3.14159}},
        line_touch   = Line{{0.0, std::sqrt(2.0)}, {std::sqrt(2.0), 0.0}},
        line_faraway = Line{{-10.0, 10.0}, {100.0, 500.0}},
        circle       = Circle{origin, 1.0}
    ;

    for (const auto &line : {line_in, line_out, line_touch}) {
        // when
        const auto got = intersections::GetIntersectPoint(circle, line);
        // then
        EXPECT_TRUE(got.has_value());              // check expected
        EXPECT_TRUE(got.value().has_value());      // check optional
        EXPECT_EQ(got.value().value(), x45degree); // check point
    }

    // when
    const auto got_faraway = intersections::GetIntersectPoint(circle, line_faraway);
    //then
    EXPECT_TRUE(got_faraway.has_value());
    EXPECT_FALSE(got_faraway.value().has_value());
}

TEST(TestIntersections, TestGetIntersectPointSpec) {
    // given
    constexpr Point2D origin{0.0, 0.0};
    constexpr Shape
        x_const0 = Line{{0.0, -1.1}, {0.0, 0.777}},
        x_const1 = Line{{1.0, -3.74}, {1.0, 44.5555}},
        y_const1 = Line{{-3.333, -1.0}, {1e8, -1.0}},
        circle   = Circle{origin, 1.0}
    ;

    // when
    const auto got_x0 = intersections::GetIntersectPoint(x_const0, circle);
    // then
    EXPECT_TRUE(got_x0.has_value());         // check expected
    EXPECT_TRUE(got_x0.value().has_value()); // check optional
    EXPECT_TRUE((got_x0.value().value() == Point2D{0.0, 1.0} || got_x0.value().value() == Point2D{0.0, -1.0})); // check points

    // when
    const auto got_x1 = intersections::GetIntersectPoint(x_const1, circle);
    // then
    EXPECT_TRUE(got_x1.has_value());         // check expected
    EXPECT_TRUE(got_x1.value().has_value()); // check optional
    EXPECT_EQ(got_x1.value().value(), (Point2D{1.0, 0.0})); // check point

    // when
    const auto got_y1 = intersections::GetIntersectPoint(y_const1, circle);
    // then
    EXPECT_TRUE(got_y1.has_value());         // check expected
    EXPECT_TRUE(got_y1.value().has_value()); // check optional
    EXPECT_EQ(got_y1.value().value(), (Point2D{0.0, -1.0})); // check point
}
