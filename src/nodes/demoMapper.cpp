#include "Mapper.hpp"

int main(int argc, char *argv[])
{
    VoxelGrid voxel_grid(0.25);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapper>(&voxel_grid));
    rclcpp::shutdown();
    return 0;
}