#pragma once
#include "path.h"
#include "obstacle.h"

class Intersection {
    Path _path;
    Obstacle _obstacle;
    const int MAX_ITERATIONS = 512;
public: 
    Intersection(const Path& path, const Obstacle& obstacle);
    bool intersects() const; 
    bool intersectsAnalytic() const;
    static std::vector<Eigen::Vector2d> intersectionLineCircle(
        const Eigen::Vector2d& line_start, 
        const Eigen::Vector2d& line_end, 
        const Eigen::Vector2d& circle_center, 
        double circle_radius
    );
    static bool intersects(const Path& path, const Obstacle& obstacle); 
};
