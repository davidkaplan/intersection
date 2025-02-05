#include <Eigen/Dense> 
#include <vector>
#include <iostream>

struct Obstacle { 
    const std::vector<Eigen::Vector2d> vertices; 
    const float height; 
    //const Eigen::Vector2d bbox[2];
    std::array<Eigen::Vector2d, 2> bbox;
    Obstacle(const std::vector<Eigen::Vector2d>& vertices, float height) 
        : vertices(vertices), height(height) 
    {
        bbox[0](Eigen::Vector2d(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
        bbox[1](Eigen::Vector2d(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()));

        for (const auto& vertex : vertices) {
            bbox[0] = bbox[0].cwiseMin(vertex).matrix();
            if (vertex.x() < bbox[0][0]) {
                bbox[0][0] = vertex[0];
            }
            bbox[0] = bbox[0].cwiseMin(vertex);
            bbox[1] = bbox[1].cwiseMax(vertex);
        }
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
        Eigen::Vector3d prev_point(vertices[0].x(), vertices[0].y(), 0);
        Eigen::Vector3d internal_point(point.x(), point.y(), 0);
        for ( int i = 1; i < vertices.size(); i++ )
        { 
            Eigen::Vector3d curr_point(vertices[i].x(), vertices[i].y(), 0);
            Eigen::Vector3d perim = curr_point - prev_point;
            Eigen::Vector3d radial = curr_point - internal_point;
            Eigen::Vector3d cross = radial.cross(perim);
            // Z component will always be negative if point is inside
            if ( cross.z() > 0 )
            {
                return false;
            }
            prev_point = curr_point;
        }
        return true;
    }

    bool containsPoint( Eigen::Vector2d&& point ) const
    {
        return containsPoint( point );
    }
}; 