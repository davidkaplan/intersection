#include "path.h"
#include "obstacle.h"

class Intersection {
    Path _path;
    Obstacle _obstacle;
public: 
    Intersection(const Path& path, const Obstacle& obstacle);
    static bool intersects(const Path& path, const Obstacle& obstacle); 
};


class BoundingBox2D {
private:
    Eigen::Vector2d _min; 
    Eigen::Vector2d _max; 

public: 
    BoundingBox2D() : 
        _min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
        _max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()) 
        {}; 

    BoundingBox2D(std::vector<Eigen::Vector2d> vertices) : BoundingBox2D() 
    {
        for (const auto& vertex : vertices) {
            _min = _min.cwiseMin(vertex);
            _max = _max.cwiseMax(vertex);
        }
    };
    
    bool intersects(const BoundingBox2D& other) const { 
        return _min.x() <= other._max.x() && _max.x() >= other._min.x() && 
            _min.y() <= other._max.y() && _max.y() >= other._min.y(); 
    }

    bool contains(const Eigen::Vector2d& point) const { 
        return _min.x() <= point.x() && _max.x() >= point.x() && 
            _min.y() <= point.y() && _max.y() >= point.y(); 
    }

    bool contains(const BoundingBox2D& other) const { 
        return _min.x() <= other._min.x() && _max.x() >= other._max.x() && 
            _min.y() <= other._min.y() && _max.y() >= other._max.y(); 
    }

    bool contains(const std::vector<Eigen::Vector2d>& vertices) const { 
        for (const auto& vertex : vertices) { 
            if (!contains(vertex)) { 
                return false; 
            }
        }
        return true; 
    }

    void addPoint(const Eigen::Vector2d& point) {
        _min = _min.cwiseMin(point);
        _max = _max.cwiseMax(point);
        std::cout << "BBOX DEBUG: adding point: " << point.transpose() << std::endl;
        std::cout << "BBOX DEBUG: current bbox values: " << *this << std::endl;
    }

    Eigen::Vector2d getMin() const {
        return _min;
    }

    Eigen::Vector2d getMax() const {
        return _max;
    }

    static BoundingBox2D fromPath(const Path& path) {
        float start_angle = path.getStartAngle();
        float end_angle = path.getEndAngle();
        Eigen::Vector2d center = path.getCenter();
        float radius = path.getRadius();
        float delta = end_angle - start_angle;

        // if the angle is greater than 360, then we have a full circle
        if (delta >= 360) {
            return BoundingBox2D({center + Eigen::Vector2d(radius, 0), center - Eigen::Vector2d(radius, 0), 
                center + Eigen::Vector2d(0, radius), center - Eigen::Vector2d(0, radius)});
        }

        BoundingBox2D bbox = BoundingBox2D();

        // first make sure start angle is positive
        if ( start_angle < 0 ) {
            start_angle = start_angle + 360*(ceil(abs(start_angle)/360));
            end_angle = start_angle + delta;
        }
        // constrain to 0-360
        start_angle = fmod(start_angle, 360);
        end_angle = fmod(end_angle, 360);
        //end_angle = start_angle + delta;
        bbox.addPoint(path.getPointByAngle(start_angle));
        bbox.addPoint(path.getPointByAngle(end_angle));
        //bool passes_through[] = {false, false, false, false}; // N, E, S, W
        if ( end_angle <= start_angle  ) {
            //passes_through[0] = true;
            // because we've clamped start angle to 0-360, then by definition if 
            // end angle is less than start it means we've gone through north.
            std::cout << "adding north" << std::endl;
            bbox.addPoint(path.getPointByAngle(0));
        }
        if ( start_angle <= 90 && ( end_angle <= start_angle || end_angle >= 90 ) ) {
            //passes_through[1] = true;
            std::cout << "adding east" << std::endl;
            bbox.addPoint(path.getPointByAngle(90));
        }
        if ( start_angle <= 180 && ( end_angle <= start_angle || end_angle >= 180 ) ) {
            //passes_through[1] = true;
            std::cout << "adding south" << std::endl;
            bbox.addPoint(path.getPointByAngle(180));
        }
        if ( start_angle <= 270 && ( end_angle <= start_angle || end_angle >= 270 ) ) {
            //passes_through[1] = true;
            std::cout << "adding west" << std::endl;
            bbox.addPoint(path.getPointByAngle(270));
        }
        return bbox;
    }

    friend std::ostream& operator<<(std::ostream& os, const BoundingBox2D bbox) {
        os << "BoundingBox:" << std::endl;
        os << "  min: " << bbox._min.transpose() << std::endl;
        os << "  max: " << bbox._max.transpose() << std::endl;
        return os; 
    }
};