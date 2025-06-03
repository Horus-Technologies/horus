#include "FSM.hpp"

int main(int argc, char * argv[])
{
  VoxelGrid voxel_grid(0.5);

  rclcpp::init(argc, argv);
  auto voxel_mapper = std::make_shared<VoxelMapper>(&voxel_grid);
  auto global_planner = std::make_shared<GlobalPlanner>(&voxel_grid);
  auto local_planner = std::make_shared<LocalPlanner>(&voxel_grid);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(voxel_mapper);
  executor.add_node(global_planner);
  executor.add_node(local_planner);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}