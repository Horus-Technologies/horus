#include "ssMapper.hpp"

int main(int argc, char *argv[])
{
    std::array<float,3> mapOffset({0,0,0});
    CostMap costMap(1, mapOffset);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ssMapper>(&costMap));
    rclcpp::shutdown();
    return 0;
}