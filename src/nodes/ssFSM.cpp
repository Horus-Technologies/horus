#include "ssFSM.hpp"

int main(int argc, char * argv[])
{
  CostMap costMap(0.5);

  rclcpp::init(argc, argv);
  auto mapper = std::make_shared<ssMapper>(&costMap);
  auto globalPlanner = std::make_shared<ssGlobalPlanner>(&costMap);
  auto localPlanner = std::make_shared<ssLocalPlanner>(&costMap);
  // auto trajectoryController = std::make_shared<ssTrajectoryController>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mapper);
  executor.add_node(globalPlanner);
  executor.add_node(localPlanner);
  // executor.add_node(trajectoryController);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}