#include "intersection.h"

#include <Eigen/Dense> 
#include <vector> 
#include <limits>

class Polygon2D {
    const std::vector<Eigen::Vector2d> _vertices; 
    const BoundingBox2D _bbox; 

    public: 
    Polygon2D(const std::vector<Eigen::Vector2d>& vertices) : _vertices(vertices), 
        _bbox(vertices) 
        {}; 
};

Intersection::Intersection(const Path& path, const Obstacle& obstacle) :
    _path(path),
    _obstacle(obstacle)
{
}

bool Intersection::intersects(const Path& path, const Obstacle& obstacle) { 
    // if max height of obstacle is less than min height of path, return false
    if (  obstacle.height < path.getMinHeight() )
    {
        return false;
    }

    // truncate path by height of obstacle
    Path truncated_path = path.truncateByHeight(obstacle.height);

    // determine some reasonable precision, from the smallest bbox dimension of the obstacle, and the path radius


    // recursively split path into segments, testing each one's bounding box agains the obstacle's bounding box
    // if bounding boxes intersect, we test point internality with the cross product method
    std::vector<Path> paths = {path};

    // test start and end point of original path for bounds... algorithm will only check midpoints
    Eigen::Vector2d start_point = path.getStartPoint();
    Eigen::Vector2d end_point = path.getEndPoint();
    if ( obstacle.containsPoint(start_point) || obstacle.containsPoint(end_point) )
    {
        return true;
    }
    

    return true; 
}

