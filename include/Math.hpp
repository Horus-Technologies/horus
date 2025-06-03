
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Math{
    std::vector<Eigen::Vector3f> sphere_line_intersection(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& circle_center, float& circle_radius);
    std::vector<Eigen::Vector3f> sphere_segment_intersection(Eigen::Vector3f& point0, Eigen::Vector3f& point1, Eigen::Vector3f& circle_center, float& circle_radius);
    Eigen::Vector3f point_line_projection(Eigen::Vector3f& point, Eigen::Vector3f& line0, Eigen::Vector3f& line1);
    Eigen::Vector3f point_segment_projection(Eigen::Vector3f& point, Eigen::Vector3f& seg0, Eigen::Vector3f& seg1);
    Eigen::Vector3d get_bezier_point(std::vector<Eigen::Vector3d>& points, double s);
}