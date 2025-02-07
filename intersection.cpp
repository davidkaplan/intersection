#include "intersection.h"

#include <Eigen/Dense> 
#include <vector> 
#include <limits>

/**
 * Class for testing intersections between a path (arc) and an obstacle (polygon).
 * We provide two methods: an analytic method (working), and a discreet method (in progress, not working).
 */
Intersection::Intersection(const Path& path, const Obstacle& obstacle) :
    _path(path),
    _obstacle(obstacle)
{
}

/**
 * Discreet method for testing intersections between a path and an obstacle.
 * Didn't quite get this working, but the idea here is to use a breadth first search tree to
 * split an arc in half and test the two children for intersection.
 * We accelerate by first testing the bounding boxes of the arc with the bounds of the obstacle, and
 * only then test the midpoint of the arc against the vertices of the obstacle
 */
bool Intersection::intersectsDiscreet() const { 
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

/**
 * Quadratic solution for finding intersections between a line segment and a circle in 2D.
 * Will return 0, 1, or 2 intersection points in a std::vector<Eigen::Vector2d>.
 */
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

/**
 * Analytic method for testing intersections between a path and an obstacle.
 * This method is working. Here we use the quadratical solution to find the exact intersection points.
 */
bool Intersection::intersectsAnalytic() const
{
    // Check top face of obstacle first, here we must test if the point is inside the polygon
    double angle_at_obstacle_height = _path.getAngleAtHeight(_obstacle.height);
    if ( angle_at_obstacle_height >= _path.getStartAngle() && angle_at_obstacle_height <= _path.getEndAngle() )
    {
        Eigen::Vector2d point_at_top = _path.getPointByAngle(angle_at_obstacle_height);
        if ( _obstacle.containsPoint(point_at_top) )
        {
            return true;
        }
    }

    // Clamp our angles to 0-360.  This allows us to handle cases of any arbitrary rotations beyond (+/-)360 degrees.
    double start_angle = Path::getConstrainedAngle(_path.getStartAngle());
    double end_angle = Path::getConstrainedAngle(_path.getEndAngle());

    // First, loop through each line segment created by two adjoining vertices of the polygon
    for ( int i = 0; i < _obstacle.vertices.size(); i++ )
    { 
        int prev_index = i == 0 ? _obstacle.vertices.size() - 1 : i - 1;
        Eigen::Vector2d line_start = _obstacle.vertices[prev_index];
        Eigen::Vector2d line_end = _obstacle.vertices[i];

        // Find intersection points between the line segment and the circle
        std::vector<Eigen::Vector2d> points = intersectionLineCircle(line_start, line_end, _path.getCenter(), _path.getRadius());
        
        // For each solution point that exists, check that it's height is below the obstacle height.
        for ( auto point : points )
        {
            // swap x and y args to atan because of our coordinate system (angle is measured clockwise from the y-axis)
            double angle = atan2(point.x(), point.y()) * 180 / M_PI;
            // Constrain angle to 0-360
            angle = Path::getConstrainedAngle(angle);
            if ( start_angle > end_angle ) // we've gone through north, therefore use OR logic.
            {
                if ( angle >= start_angle || angle <= end_angle )
                {
                    if ( _path.getPointByAngle3D(angle).z() <= _obstacle.height )
                    {
                        return true;
                    }
                }
            }
            else // else handle angle normally, using AND logic.
            {
                if ( angle >= start_angle && angle <= end_angle )
                {
                    if ( _path.getPointByAngle3D(angle).z() <= _obstacle.height )
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/**
 * Static method for testing intersections between a path and an obstacle.
 * Hardcoded to use the analytic method. Can be swapped for discreet method for further development.
 */
bool Intersection::intersects(const Path& path, const Obstacle& obstacle)
{ 
    // if max height of obstacle is less than min height of path, return false
    if ( obstacle.height < path.getMinHeight() )
    {
        std::cout << "obstacle height less than start of path, returning false." << std::endl;
        return false;
    }
    Intersection i(path, obstacle);
    return i.intersectsAnalytic();
}
