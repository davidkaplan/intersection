#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "intersection.h"

class IntersectionTest : public ::testing::Test {
protected:
    Path path_1;
    Path path_2;
    Path path_3;
    Obstacle obstacle_1;
    Obstacle obstacle_2;
    Obstacle obstacle_3;
    IntersectionTest() :
        path_1({0, 0}, 15, 0, 90, 25, 15),
        path_2({10, 10}, 5, -45, 225, 18, 30),
        path_3({10, -10}, 15, -15, 15, 60, 10),
        obstacle_1(std::vector<Eigen::Vector2d>({{10, 10}, {10, 0}, {0, 10}}), 30),
        obstacle_2(std::vector<Eigen::Vector2d>({{10, 0}, {10, 10}, {20, 10}, {20, 0}}), 20),
        obstacle_3(std::vector<Eigen::Vector2d>({{0, 10}, {0, 20}, {10, 20}, {10, 10}}), 20)
    {}

    /*
    void SetUp() override {
    }
    void TearDown() override {
    }
    */
};

TEST(PathTest, Test_Path)
{
    Path path({0.5, 0.5}, 1, 0, 180, 0, 1);
    EXPECT_TRUE(path.getPointByAngle(0).isApprox(Eigen::Vector2d(0.5, 1.5)));
    auto p1 = path.getPointByAngle(90);
    auto p2 = Eigen::Vector2d(1.5, 0.5);
    EXPECT_TRUE(p1.isApprox(p2));
    Path path_1({0, 0}, 15, 0, 90, 25, 15);
    EXPECT_TRUE(path_1.getStartPoint().isApprox(Eigen::Vector2d(0, 15)));
    EXPECT_TRUE(path_1.getEndPoint().isApprox(Eigen::Vector2d(15, 0)));
}

TEST(BoundingBoxTest, Test_BoundingBox)
{
    double eigenEpsilon = Eigen::NumTraits<double>::epsilon();
    Path path_2({0, 0}, 1, -45, 45, 0, 1);
    BoundingBox2D bbox_2 = path_2.computeBBox();
    // Remember 0 is +Y, have to flip x and y arguments
    Eigen::Vector2d pMax(Eigen::Vector2d(sin(45 * M_PI / 180), 1));
    Eigen::Vector2d pMin(Eigen::Vector2d(sin(-45 * M_PI / 180), cos(-45 * M_PI / 180)));
    EXPECT_TRUE(bbox_2.getMax().isApprox(pMax));
    EXPECT_TRUE(bbox_2.getMin().isApprox(pMin));
}

TEST(ObstacleTest, Test_Obstacle)
{
    Obstacle obstacle_1(std::vector<Eigen::Vector2d>({{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}), 30);
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({0, 0})));
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({0.99, 0.99})));
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({-0.99, -0.99})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({2, 0})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({1.01, 1.01})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({-1.01, -1.01})));
}

TEST(AnalyticalIntersectionTest, Test_AnalyticalIntersection)
{
    Eigen::Vector2d line_start({-2, 0});
    Eigen::Vector2d line_end({2, 0});
    Eigen::Vector2d circle_center({0, 0});
    double circle_radius = 1;
    std::vector<Eigen::Vector2d> points = Intersection::intersectionLineCircle(line_start, line_end, circle_center, circle_radius);
    EXPECT_TRUE(points.size() == 2);
    EXPECT_TRUE(points[0].isApprox(Eigen::Vector2d({1, 0})));
    EXPECT_TRUE(points[1].isApprox(Eigen::Vector2d({-1, 0})));

    Path path_1({0, 0}, 15, 0, 90, 25, 15);
    Obstacle obstacle_2(std::vector<Eigen::Vector2d>({{10, 0}, {10, 10}, {20, 10}, {20, 0}}), 20);
    EXPECT_TRUE(Intersection::intersects(path_1, obstacle_2));
}

TEST_F(IntersectionTest, Test_Path1) {
    EXPECT_FALSE(Intersection::intersects(path_1, obstacle_1));
    EXPECT_TRUE(Intersection::intersects(path_1, obstacle_2));
    EXPECT_FALSE(Intersection::intersects(path_1, obstacle_3));
}

TEST_F(IntersectionTest, Test_Path2) {
    EXPECT_TRUE(Intersection::intersects(path_2, obstacle_1));
    EXPECT_FALSE(Intersection::intersects(path_2, obstacle_2));
    EXPECT_TRUE(Intersection::intersects(path_2, obstacle_3));

}

TEST_F(IntersectionTest, Test_Path3) {
    EXPECT_FALSE(Intersection::intersects(path_3, obstacle_1));
    EXPECT_TRUE(Intersection::intersects(path_3, obstacle_2));
    EXPECT_FALSE(Intersection::intersects(path_3, obstacle_3));
}
