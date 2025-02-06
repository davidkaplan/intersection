#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include "intersection.h"

//double EPSILON = 2 * Eigen::NumTraits<float>::epsilon();

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
    std::cout << path << std::endl;
    EXPECT_TRUE(path.getPointByAngle(0).isApprox(Eigen::Vector2d(0.5, 1.5)));
    std::cout << path.getPointByAngle(90) << std::endl;
    std::cout << Eigen::Vector2d(1.5, 0.5) << std::endl;
    auto p1 = path.getPointByAngle(90);
    auto p2 = Eigen::Vector2d(1.5, 0.5);
    EXPECT_TRUE(p1.isApprox(p2, FLT_EPSILON));
    //EXPECT_NEAR(p1.x(), p2.x(), FLT_EPSILON);
    //EXPECT_NEAR(p1.y(), p2.y(), FLT_EPSILON);
}

TEST(BoundingBoxTest, Test_BoundingBox)
{
    /*Path path_1({0, 0}, 1, 0, 180, 0, 1);
    BoundingBox2D bbox = BoundingBox2D::fromPath(path_1);
    std::cout << bbox << std::endl;
    Eigen::Vector2d pMax = bbox.getMax();
    Eigen::Vector2d pMin = bbox.getMin();
    EXPECT_TRUE(pMax.isApprox(Eigen::Vector2d(1, 1), FLT_EPSILON));
    EXPECT_TRUE(pMin.isApprox(Eigen::Vector2d(0, -1), FLT_EPSILON));*/

    double eigenEpsilon = Eigen::NumTraits<double>::epsilon();
    std::cout << "Eigen epsilon: " << eigenEpsilon << std::endl;
    std::cout << "FLT_EPSILON: " << FLT_EPSILON << std::endl;
    std::cout << "DBL_EPSILON: " << DBL_EPSILON << std::endl;
    //std::cout << "EPSILON: " << EPSILON << std::endl;

    Path path_2({0, 0}, 1, -45, 45, 0, 1);
    BoundingBox2D bbox_2 = path_2.computeBBox();
    // Remember 0 is +Y, have to flip x and y arguments
    Eigen::Vector2d pMax(Eigen::Vector2d(sin(45 * M_PI / 180), 1));
    Eigen::Vector2d pMin(Eigen::Vector2d(sin(-45 * M_PI / 180), cos(-45 * M_PI / 180)));
    EXPECT_TRUE(bbox_2.getMax().isApprox(pMax, DBL_EPSILON));
    //EXPECT_NEAR(pMin.x(), bbox_2.getMin().x(), FLT_EPSILON);
    //EXPECT_NEAR(pMin.y(), bbox_2.getMin().y(), FLT_EPSILON);
    EXPECT_TRUE(bbox_2.getMin().isApprox(pMin));
}

TEST(ObstacleTest, Test_Obstacle)
{
    Obstacle obstacle_1(std::vector<Eigen::Vector2d>({{1, 1}, {1, -1}, {-1, -1}, {-1, 1}}), 30);
    //std::cout << obstacle_1 << std::endl;
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({0, 0})));
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({0.99, 0.99})));
    EXPECT_TRUE(obstacle_1.containsPoint(Eigen::Vector2d({-0.99, -0.99})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({2, 0})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({1.01, 1.01})));
    EXPECT_FALSE(obstacle_1.containsPoint(Eigen::Vector2d({-1.01, -1.01})));
}

TEST_F(IntersectionTest, Test_Path1) {
    EXPECT_EQ(Intersection::intersects(path_1, obstacle_1), false);
    EXPECT_EQ(Intersection::intersects(path_1, obstacle_2), true);
    EXPECT_EQ(Intersection::intersects(path_1, obstacle_3), false);
}

TEST_F(IntersectionTest, Test_Path2) {
    EXPECT_EQ(Intersection::intersects(path_2, obstacle_1), true);
    EXPECT_EQ(Intersection::intersects(path_2, obstacle_2), false);
    EXPECT_EQ(Intersection::intersects(path_2, obstacle_3), true);

}

TEST_F(IntersectionTest, Test_Path3) {
    EXPECT_EQ(Intersection::intersects(path_3, obstacle_1), false);
    EXPECT_EQ(Intersection::intersects(path_3, obstacle_2), true);
    EXPECT_EQ(Intersection::intersects(path_3, obstacle_3), false);
}
