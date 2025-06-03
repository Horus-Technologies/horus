
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Math{
    Eigen::Vector3d get_bezier_point(std::vector<Eigen::Vector3d>& points, double s);
}