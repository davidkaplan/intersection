#pragma once
#include <Eigen/Dense> 
#include <iostream>

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
            addPoint(vertex);
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
    }

    Eigen::Vector2d getMin() const {
        return _min;
    }

    Eigen::Vector2d getMax() const {
        return _max;
    }

    friend std::ostream& operator<<(std::ostream& os, const BoundingBox2D bbox) {
        os << "BoundingBox:" << std::endl;
        os << "  min: " << bbox._min.transpose() << std::endl;
        os << "  max: " << bbox._max.transpose() << std::endl;
        return os; 
    }
};