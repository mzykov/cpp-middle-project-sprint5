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
}
