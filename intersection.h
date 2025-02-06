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
    static bool intersects(const Path& path, const Obstacle& obstacle); 
};
