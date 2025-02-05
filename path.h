#include <Eigen/Dense> 
#include <vector> 
#include <iostream>
#include <math.h>

class Path { 
    const Eigen::Vector2d _center; 
    const double _radius; 
    const double _start_angle; 
    const double _end_angle;
    const double _start_height; 
    const double _end_height; 

    public: 
    Path(const Eigen::Vector2d& center, double radius, double start_angle, 
        double end_angle, double start_height, double end_height) : _center(center), 
        _radius(radius), 
        _start_angle(start_angle), 
        _end_angle(end_angle), 
        _start_height(start_height), 
        _end_height(end_height) 
        {
            // sanity checks:
            if (radius <= 0) {
                throw std::invalid_argument("radius must be positive");
            }
            if (start_angle == end_angle) {
                throw std::invalid_argument("start and end angles cannot be equal, arc must have some length");
            }
            if (end_angle < start_angle) {
                throw std::invalid_argument("end angle must be greater than start angle, only clockwise paths allowed");
            }
        };

    std::vector<Path> split(double t = 0.5)
    {
        double delta_angle = _end_angle - _start_angle;
        double delta_height = _end_height - _start_height;
        double split_angle= _start_angle + ( t * delta_angle);
        double split_height = _start_height + ( t * delta_height);
        Path p1 = Path(_center, _radius, _start_angle, split_angle, _start_height, split_height);
        Path p2 = Path(_center, _radius, split_angle, _end_height, split_height, _end_height);
        return std::vector<Path>{p1, p2};
    }

    Path truncateByHeight(double max_height) const {
        double new_start_angle = _start_angle;
        double new_end_angle = _end_angle;
        double new_start_height = _start_height;
        double new_end_height = _end_height;
        double delta_theta = _end_angle - _start_angle;
        double delta_height = _end_height - _start_height;
        double rate = delta_height / delta_theta; // rate of change of height per degree

        if ( delta_height > 0 )
        {
            if ( max_height < _end_height )
            {
                new_end_height = max_height;
                new_end_angle = _start_angle + (new_end_height - _start_height) / rate;
            }
        }
        else if ( delta_height < 0 )
        {
            if ( max_height > _start_height )
            {
                new_start_height = max_height;
                new_start_angle = _start_angle + (new_start_height - _start_height) / rate;
            }
        }
    
        return Path(_center, _radius, new_start_angle, new_end_angle, new_start_height, new_end_height);
    }

    Eigen::Vector2d getPointByAngle(double angle) const {
        // flip x and y because our angle is measured clockwise from the y-axis
        double rad = angle * M_PI / 180;
        return _center + Eigen::Vector2d( _radius * sin(rad), _radius * cos(rad));
    }

    Eigen::Vector2d getPointByUnitDistance(double unit_distance) const {
        double angle = _start_angle + unit_distance * (_end_angle - _start_angle);
        return getPointByAngle(angle);
    }

    Eigen::Vector2d getStartPoint() const {
        return getPointByAngle(_start_angle);
    }

    Eigen::Vector2d getEndPoint() const {
        return getPointByAngle(_end_angle);
    }

    Eigen::Vector2d getMidPoint() const {
        return getPointByUnitDistance(0.5);
    }

    double getArcDistance() const {
        return M_PI * _radius * (_end_angle - _start_angle) / 180;
    }

    double getChordDistance() const {
        return _radius * sqrt(2 - 2 * cos((_end_angle - _start_angle) * M_PI / 180));
    }

    friend std::ostream& operator<<(std::ostream& os, const Path& obj) {
        os << "Path:" <<  std::endl;
        os << "  center: " << obj._center.transpose() << std::endl;
        os << "  radius: " << obj._radius << std::endl;
        os << "  start_angle: " << obj._start_angle << std::endl;
        os << "  end_angle: " << obj._end_angle << std::endl;
        os << "  start_height: " << obj._start_height << std::endl;
        os << "  end_height: " << obj._end_height << std::endl;
        return os;
    }

    Eigen::Vector2d getCenter() const {
        return _center;
    }
    double getRadius() const {
        return _radius;
    }
    double getStartAngle() const {
        return _start_angle;
    }
    double getEndAngle() const {
        return _end_angle;
    }

    double getMinHeight() const {
        return std::min(_start_height, _end_height);
    }

}; 