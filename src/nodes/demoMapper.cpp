#include "ssMapper.hpp"

int main(int argc, char *argv[])
{
    std::array<float,3> mapOffset({-2,-2,0});
    CostMap costMap(0.25, mapOffset);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ssMapper>(&costMap));
    rclcpp::shutdown();
    return 0;
}