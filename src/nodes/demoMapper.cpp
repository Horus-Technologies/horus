#include "ssMapper.hpp"

int main(int argc, char *argv[])
{
    openvdb::FloatGrid::Ptr map;
    openvdb::initialize();
    
    // Create a floating-point grid with a background value of 0
    map = openvdb::FloatGrid::create(/*background value=*/0.0);
    
    // Set a name for the grid (optional)
    map->setName("MyVoxelGrid");

    // Create a new linear transform with the desired voxel size
    openvdb::math::Transform::Ptr newTransform =
        openvdb::math::Transform::createLinearTransform(1.0f);

    // Set the grid's transform to the new transform
    map->setTransform(newTransform);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ssMapper>(map));
    rclcpp::shutdown();
    return 0;
}