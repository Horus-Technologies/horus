#include "ssLocalPlanner.hpp"

using namespace std::chrono_literals;

ssLocalPlanner::ssLocalPlanner(CostMap* costMap)
: Node("ssLocalPlanner"), _count(0), _costMap(costMap)
{

    // Callback group
    // auto exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // rclcpp::SubscriptionOptions options;
    // options.callback_group = exclusive_group;
    
    // Subscribing
    rclcpp::QoS qos(rclcpp::KeepLast(1)); 
    qos.best_effort();
    _subscriberDrone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ap/pose/filtered", qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp) {
        this->callback_drone(poseStamp);
    });

    _subscriberGoal = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/global_goal", 10,
      [this](const std_msgs::msg::UInt16MultiArray::SharedPtr goal) {
          this->callback_goal(goal);
      });

    // Publishing
    _publisher = this->create_publisher<nav_msgs::msg::Path>("waypoints", 10); // waypoints with stamped pose
    _timer = this->create_wall_timer(500ms, std::bind(&ssLocalPlanner::run, this));
    _publisher_path_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("path/markers", 10);

}

    // Publisher function for path - on a timer callback
void ssLocalPlanner::run()
{
  // std::lock_guard<std::mutex> lock(_mutex);
  // _costMap->addObstacle({3.2, 5, 0}, {4, 5.2, 2});
  // _costMap->addObstacle({0.2, 4, 0}, {1, 4.2, 2});
  // Update _start
  _start = _costMap->getVoxelIndices({
    _lastPoseDrone.pose.position.x,
    _lastPoseDrone.pose.position.y,
    _lastPoseDrone.pose.position.z
  });
  // temporarily until costmap frame is figured out
  if (_start[0] < 0) {_start[0] = 0;}
  if (_start[1] < 0) {_start[1] = 0;}
  if (_start[2] < 0) {_start[2] = 0;}

  std::unique_ptr<int[]> came_from = Search::runBreadthFirst(*_costMap, _start, _goal);

  // Obtain path
  std::array<int,3> current = _goal;
  std::vector<std::array<int,3>> path;
  while(current != _start)
  {
    path.push_back(current);
    // RCLCPP_INFO(this->get_logger(), "Current: %d %d %d"
    //   , current[0], current[1], current[2]);
    std::array<int,3> prev = _costMap->unflatten(
      came_from[_costMap->flatten(current)]);
    current = prev;
    
  }
  path.push_back(_start); // append start voxel
  
  std::reverse(path.begin(),path.end()); // start --> goal
  visualizePath(path);
  Search::cleanPath(*_costMap, path);
  Search::cleanPath(*_costMap, path);

  _lastPath.poses.clear();
  
  // compute point on _start voxel that intersects with first 
  double totalPathTime = 0.0;
  int i = 0;
  std::array<int,3> prevVoxelIndices = path.front();
  for(std::array<int,3> voxelIndices : path)
  {
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = _costMap->getVoxelPosition(voxelIndices)[0];
    pose.pose.position.y = _costMap->getVoxelPosition(voxelIndices)[1];
    pose.pose.position.z = _costMap->getVoxelPosition(voxelIndices)[2];
    pose.pose.orientation.x = 0.00081;
    pose.pose.orientation.y = 0.000069;
    pose.pose.orientation.z = 0.703855;
    pose.pose.orientation.w = 0.710343;
    if (i>0){
      totalPathTime = totalPathTime + 1000 * sqrt(
        pow(pose.pose.position.x - _costMap->getVoxelPosition(prevVoxelIndices)[0], 2) +
        pow(pose.pose.position.y - _costMap->getVoxelPosition(prevVoxelIndices)[1], 2) +
        pow(pose.pose.position.z - _costMap->getVoxelPosition(prevVoxelIndices)[2], 2));
      pose.header.stamp.sec = static_cast<int32_t>(totalPathTime);
    }
    else{
      pose.header.stamp.sec = 0.0;
    }

    _lastPath.poses.push_back(pose);
    i++;
    prevVoxelIndices = voxelIndices;
  }
  
  _lastPath.header.stamp = this->now();
  _lastPath.header.frame_id = "odom";
  _publisher->publish(_lastPath);
}

void ssLocalPlanner::visualizePath(std::vector<std::array<int,3>>& path)
{
  visualization_msgs::msg::MarkerArray path_markers;
  int markerId = 0;
  for (std::array<int,3> indices : path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "path_markers";
    marker.id = markerId;
    markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose
    std::array<float, 3> pos = _costMap->getVoxelPosition(indices);
    marker.pose.position.x = pos[0];
    marker.pose.position.y = pos[1];
    marker.pose.position.z = pos[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set the scale
    const float scale = _costMap->getScale();
    marker.scale.x = scale/2;
    marker.scale.y = scale/2;
    marker.scale.z = scale/2;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Add the marker to the array
    path_markers.markers.push_back(marker);
  }
  _publisher_path_markers->publish(path_markers);
}

void ssLocalPlanner::callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp)
{
  // std::lock_guard<std::mutex> lock(_mutex);
  _lastPoseDrone = *poseStamp;
  // RCLCPP_INFO(this->get_logger(),"Drone Pose Received: %f %f %f", 
  // _lastPoseDrone.pose.position.x,
  // _lastPoseDrone.pose.position.y,
  // _lastPoseDrone.pose.position.z);
 
}

void ssLocalPlanner::callback_goal(const std_msgs::msg::UInt16MultiArray::SharedPtr goal)
{
  // std::lock_guard<std::mutex> lock(_mutex);
  std::vector<uint16_t> goalData = goal->data;
  _goal = {goalData[0],goalData[1],goalData[2]};

  // RCLCPP_INFO(this->get_logger(),"Goal received from GlobalPlanner: %d %d %d", goalData[0],goalData[1],goalData[2]);
}
