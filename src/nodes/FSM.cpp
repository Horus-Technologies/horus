#include "FSM.hpp"

int main(int argc, char * argv[])
{
  VoxelGrid voxel_grid(0.25);

  rclcpp::init(argc, argv);
  auto mapper = std::make_shared<Mapper>(&voxel_grid);
  auto global_planner = std::make_shared<GlobalPlanner>(&voxel_grid);
  auto local_planner = std::make_shared<LocalPlanner>(&voxel_grid);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(global_planner);
  executor.add_node(local_planner);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}