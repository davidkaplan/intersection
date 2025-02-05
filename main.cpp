#include "intersection.h"

int main(int argc, char **argv) {
    Eigen::Vector2d center(0, 0);
    Path path({10, -10}, 15, -15, 15, 60, 10);
    Obstacle obstacle(std::vector<Eigen::Vector2d>({{10, 10}, {10, 0}, {0, 10}}), 30);
    std::cout << "Running Intersection Test" << std::endl;
    std::cout << "Result: " << ((Intersection::intersects(path, obstacle)) ? "TRUE" : "FALSE") << std::endl;
    return 0;
}