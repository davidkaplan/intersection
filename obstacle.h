#pragma once
#include <Eigen/Dense> 
#include <vector>
#include <iostream>

#include "boundingbox.h"

struct Obstacle { 
    const std::vector<Eigen::Vector2d> vertices; 
    const float height; 
    //const Eigen::Vector2d bbox[2];
    BoundingBox2D bbox;
    Obstacle(const std::vector<Eigen::Vector2d>& vertices, float height) : 
        vertices(vertices), 
        height(height) ,
        bbox(vertices)
    {

    }; 

    friend std::ostream& operator<<(std::ostream& os, const Obstacle& obj) {
        os << "Obstacle:" <<  std::endl;
        os << "  height: " << obj.height << std::endl;
        os << "  vertices: " << std::endl;
        for (const auto& vertex : obj.vertices) {
            os << "    " << vertex.transpose() << std::endl;
        }
        return os;
    }

    bool containsPoint( Eigen::Vector2d& point ) const
    // Take advantage of the convex and clockwise ordered property of these polygons.
    // This will fail if the polygon is not guanteed convex, and verts are not ordered clockwise.
    {
        Eigen::Vector3d internal_point(point.x(), point.y(), 0);
        for ( int i = 0; i < vertices.size(); i++ )
        { 
            int prev_index = i - 1;
            if ( prev_index < 0 )
            {
                prev_index = vertices.size() - 1;
            }
            Eigen::Vector3d prev_point(vertices[prev_index].x(), vertices[prev_index].y(), 0);
            Eigen::Vector3d curr_point(vertices[i].x(), vertices[i].y(), 0);
            Eigen::Vector3d perim = curr_point - prev_point;
            Eigen::Vector3d radial = curr_point - internal_point;
            Eigen::Vector3d cross = radial.cross(perim);
            // Z component will always be negative if point is inside
            if ( cross.z() > 0 )
            {
                return false;
            }
        }
        return true;
    }

    bool containsPoint( Eigen::Vector2d&& point ) const
    {
        return containsPoint( point );
    }
}; 