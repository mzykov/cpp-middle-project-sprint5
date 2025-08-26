#include <gtest/gtest.h>

#include "geometry.hpp"
#include <cassert>
#include <cmath>

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
        bbox1_copy{{4.0, 4.0}, {-2.0, -3.0}},
        bbox2{{-1.1, -1.5}, {0.3, 0.9}},
        bbox3{{-3.0, -4.0}, origin}
    ;
    // when
    // then
    EXPECT_TRUE(bbox1 == bbox1_copy);
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
