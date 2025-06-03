#include "Math.hpp"

namespace Math{

    std::vector<Eigen::Vector3f> sphere_line_intersection(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& circle_center, float& circle_radius)
    {
        Eigen::Vector3f u = (point1 - point0)/(point1 - point0).norm();

        float nabla = std::pow(u.dot(point0 - circle_center),2) 
            - (std::pow((point0 - circle_center).norm() , 2) - std::pow(circle_radius,2));

        if (nabla < 0)
        {
            return {};
        }
        else if(nabla == 0){
            float d = -(u.dot(point0 - circle_center));
            Eigen::Vector3f intersection = point0 + u*d;
            return {intersection};
        }
        else{
            std::vector<Eigen::Vector3f> intersections;
            // first intersection
            float d = -(u.dot(point0 - circle_center)) - std::sqrt(nabla);
            Eigen::Vector3f intersection = point0 + u*d;
            intersections.push_back(intersection);

            // second intersection
            d = -(u.dot(point0 - circle_center)) + std::sqrt(nabla);
            intersection = point0 + u*d;
            intersections.push_back(intersection);

            return intersections;
        }
    }

    std::vector<Eigen::Vector3f> sphere_segment_intersection(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& circle_center, float& circle_radius){
        std::vector<Eigen::Vector3f> intersections_line = sphere_line_intersection(point0, point1, circle_center, circle_radius);
        std::vector<Eigen::Vector3f> intersections_segment{};

        float segment_length = (point1 - point0).norm();

        // check if each intersection point is within the segment to see if it is valid. If so, we add to returning vector
        for (Eigen::Vector3f intersection_line : intersections_line){
            float distance_to_point0 = (point0 - intersection_line).norm();
            float distance_to_point1 = (point1 - intersection_line).norm();
            
            if (distance_to_point0 <= segment_length && distance_to_point1 <= segment_length){
                intersections_segment.push_back(intersection_line);
            }
        }

        return intersections_segment;
    }

    Eigen::Vector3f point_line_projection(Eigen::Vector3f& point, Eigen::Vector3f& line0, Eigen::Vector3f& line1)
    {
        Eigen::Vector3f u = (line1 - line0)/(line1 - line0).norm();
        return line0 + ((point - line0).dot(u)) * u;
    }

    Eigen::Vector3f point_segment_projection(Eigen::Vector3f& point, Eigen::Vector3f& seg0, Eigen::Vector3f& seg1)
    {
        Eigen::Vector3f projection_line = point_line_projection(point, seg0, seg1);
        float distance_to_seg0 = (projection_line - seg0).norm();
        float distance_to_seg1 = (projection_line - seg1).norm();

        float segment_length = (seg0 - seg1).norm();

        if (distance_to_seg0 > segment_length){
            return seg1;
        }
        else if(distance_to_seg1 > segment_length){
            return seg0;
        }
        else{
            return projection_line;
        }
    }


    Eigen::Vector3d get_bezier_point(std::vector<Eigen::Vector3d>& points, double s) {
        if (points.size() <= 0) {
            return Eigen::Vector3d::Zero(); // Handle empty input
        }

        std::vector<Eigen::Vector3d> tmp = points; // Use std::vector, initialize with points

        int i = points.size() - 1;
        while (i > 0) {
            for (int k = 0; k < i; k++) {
                tmp[k] = (1.0 - s) * tmp[k] + s * tmp[k + 1]; // Correct De Casteljau's algorithm
            }
            i--;
        }
        return tmp[0];
    }
}