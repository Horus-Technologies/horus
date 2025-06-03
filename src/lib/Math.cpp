#include "Math.hpp"

namespace Math{
    Eigen::Vector3d get_bezier_point(std::vector<Eigen::Vector3d>& points, double s) {
        if (points.size() <= 0) {
            RCLCPP_INFO(this->get_logger(), "Num Ponts = 0");
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