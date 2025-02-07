#pragma once
#include <Eigen/Dense> 
#include <vector> 
#include <iostream>
#include <math.h>

#include "boundingbox.h"

class Path { 
    const Eigen::Vector2d _center; 
    const double _radius; 
    const double _start_angle; 
    const double _end_angle;
    const double _start_height; 
    const double _end_height; 
    const BoundingBox2D _bbox;

    public: 
    Path(const Eigen::Vector2d& center, double radius, double start_angle, 
        double end_angle, double start_height, double end_height) : _center(center), 
        _radius(radius), 
        _start_angle(start_angle), 
        _end_angle(end_angle), 
        _start_height(start_height), 
        _end_height(end_height),
        _bbox(computeBBox())
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

    std::vector<Path> split(double t = 0.5) const
    {
        double delta_angle = _end_angle - _start_angle;
        double delta_height = _end_height - _start_height;
        double split_angle= _start_angle + ( t * delta_angle);
        double split_height = _start_height + ( t * delta_height);
        Path p1 = Path(_center, _radius, _start_angle, split_angle, _start_height, split_height);
        Path p2 = Path(_center, _radius, split_angle, _end_angle, split_height, _end_height);
        return std::vector<Path>{p1, p2};
    }

    Path truncateByHeight(double max_height) const {
        if ( std::max(_start_height, _end_height) <= max_height )
        {
            return *this;
        }

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

    double getAngleAtHeight(double height) const {
        double delta_height = _end_height - _start_height;
        double delta_angle = _end_angle - _start_angle;
        double rate = delta_angle / delta_height; // rate of change of angle per unit height
        double angle = _start_angle + (height - _start_height) * rate;
        return angle;
    }

    Eigen::Vector2d getPointByAngle(double angle) const {
        // flip x and y because our angle is measured clockwise from the y-axis
        double rad = angle * M_PI / 180;
        return _center + Eigen::Vector2d( _radius * sin(rad), _radius * cos(rad));
    }

    Eigen::Vector3d getPointByAngle3D(double angle) const {
        Eigen::Vector2d point = getPointByAngle(angle);
        double z = _start_height + (angle - _start_angle) * (_end_height - _start_height) / (_end_angle - _start_angle);
        return Eigen::Vector3d(point.x(), point.y(), z);
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

    bool intersectsBBox(BoundingBox2D bbox) const {
        return _bbox.intersects(bbox);
    }

    static double getConstrainedAngle(double angle) {
        //  make sure angle is positive
        if ( angle < 0 ) {
            angle += 360*(ceil(abs(angle)/360));
        }
        // constrain to 0-360
        angle = fmod(angle, 360);
        return angle;
    }

    BoundingBox2D computeBBox() {
        double delta = _end_angle - _start_angle;

        // if the angle is greater than 360, then we have a full circle
        if (delta >= 360) {
            return BoundingBox2D({_center + Eigen::Vector2d(_radius, 0), _center - Eigen::Vector2d(_radius, 0), 
                _center + Eigen::Vector2d(0, _radius), _center - Eigen::Vector2d(0, _radius)});
        }

        BoundingBox2D bbox = BoundingBox2D();
        double tmp_start_angle = getConstrainedAngle(_start_angle);
        double tmp_end_angle = getConstrainedAngle(_end_angle);
        bbox.addPoint(getPointByAngle(tmp_start_angle));
        bbox.addPoint(getPointByAngle(tmp_end_angle));
        if ( tmp_end_angle <= tmp_start_angle  ) {
            // north:
            // because we've clamped start angle to 0-360, then by definition if 
            // end angle is less than start it means we've gone through north.
            bbox.addPoint(getPointByAngle(0));
        }
        if ( tmp_start_angle <= 90 && ( tmp_end_angle <= tmp_start_angle || tmp_end_angle >= 90 ) ) {
            // east
            bbox.addPoint(getPointByAngle(90));
        }
        if ( tmp_start_angle <= 180 && ( tmp_end_angle <= tmp_start_angle || tmp_end_angle >= 180 ) ) {
            // south
            bbox.addPoint(getPointByAngle(180));
        }
        if ( tmp_start_angle <= 270 && ( tmp_end_angle <= tmp_start_angle || tmp_end_angle >= 270 ) ) {
            // west
            bbox.addPoint(getPointByAngle(270));
        }
        return bbox;
    }
}; 