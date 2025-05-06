#include "ssMapper.hpp"

int main(int argc, char *argv[])
{
    CostMap costMap(0.25);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ssMapper>(&costMap));
    rclcpp::shutdown();
    return 0;
}