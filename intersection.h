#pragma once
#include "path.h"
#include "obstacle.h"

class Intersection {
    Path _path;
    Obstacle _obstacle;
public: 
    Intersection(const Path& path, const Obstacle& obstacle);
    static bool intersects(const Path& path, const Obstacle& obstacle); 
};
