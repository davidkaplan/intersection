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

bool Intersection::intersects() const { 
    // if max height of obstacle is less than min height of path, return false
    if ( _obstacle.height < _path.getMinHeight() )
    {
        std::cout << "obstacle height less than start of path, returning false." << std::endl;
        return false;
    }

    // truncate path by height of obstacle
    Eigen::Vector2d orig_start_point = _path.getStartPoint();
    Eigen::Vector2d orig_end_point = _path.getEndPoint();
    Path truncated_path = _path.truncateByHeight(_obstacle.height);

    // test start and end point of original path for bounds... algorithm will only check midpoints
    Eigen::Vector2d start_point = truncated_path.getStartPoint();
    Eigen::Vector2d end_point = truncated_path.getEndPoint();
    if ( _obstacle.containsPoint(start_point) ||  _obstacle.containsPoint(end_point) )
    {
        std::cout << "Start or end point of path found inside polygon, returning true." << std::endl;
        return true;
    }

    //
    std::queue<Path> paths;
    paths.push(truncated_path);
    int iterations = 0;
    while ( paths.size() > 0 && iterations < MAX_ITERATIONS )
    {
        iterations++;
        Path path = paths.front();
        if ( path.intersectsBBox( _obstacle.bbox) )
        {
            if ( _obstacle.containsPoint(path.getMidPoint()) )
            {
                return true;
            }
            std::vector<Path> split_path = path.split();
            paths.push(split_path[0]);
            paths.push(split_path[1]);
            
        }
        paths.pop();
    }
    return false;
}

std::vector<Eigen::Vector2d> Intersection::intersectionLineCircle(
    const Eigen::Vector2d& line_start, 
    const Eigen::Vector2d& line_end, 
    const Eigen::Vector2d& circle_center, 
    double circle_radius
) 
{
    // Translate the line and circle to the origin
    Eigen::Vector2d translated_line_start = line_start - circle_center;
    Eigen::Vector2d translated_line_end = line_end - circle_center;

    // Calculate the direction vector of the line
    Eigen::Vector2d line_direction = (translated_line_end - translated_line_start);

    // Calculate the coefficients of the quadratic equation
    double a = line_direction.squaredNorm();
    double b = 2 * (line_direction.dot(translated_line_start));
    double c = translated_line_start.squaredNorm() - circle_radius * circle_radius;

    // Calculate the discriminant
    double discriminant = b * b - 4 * a * c;

    // If the discriminant is negative, there are no intersections
    if (discriminant < 0) {
        return {};
    }

    // Calculate the two solutions of the quadratic equation
    double t1 = (-b + sqrt(discriminant)) / (2 * a);
    double t2 = (-b - sqrt(discriminant)) / (2 * a);

    // Calculate the intersection points
    std::vector<Eigen::Vector2d> intersection_points;
    
    if (t1 >= 0 && t1 <= 1) {
        intersection_points.push_back(translated_line_start + t1 * line_direction);
    }
    if (t2 >= 0 && t2 <= 1) {
        intersection_points.push_back(translated_line_start + t2 * line_direction);
    }
    return intersection_points;
}

bool Intersection::intersects(const Path& path, const Obstacle& obstacle)
{ 
    Intersection i(path, obstacle);
    return i.intersects();
}
