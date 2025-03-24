
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Math{
    Eigen::Vector3d getBezierPoint(std::vector<Eigen::Vector3d>& points, double s);
}