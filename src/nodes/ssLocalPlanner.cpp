#include "ssLocalPlanner.hpp"

using namespace std::chrono_literals;

ssLocalPlanner::ssLocalPlanner(CostMap* costMap)
: Node("ssLocalPlanner"), _count(0), _costMap(costMap)
{

    // Callback group
    auto exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = exclusive_group;
    
    // Subscribing
    // rclcpp::QoS qos(rclcpp::KeepLast(1)); 
    // qos.best_effort();
    // _subscriberDrone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    // "/ap/pose/filtered", qos,
    // [this](const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp) {
    //     this->callback_drone(poseStamp);
    // });

    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriberDrone = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        this->callback_drone(odometry);
    },options);

    _subscriberGoal = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/global_goal", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr goal) {
          this->callback_goal(goal);
      },options);

    // Publishing
    _publisher = this->create_publisher<nav_msgs::msg::Path>("waypoints", 10); // waypoints with stamped pose
    _timer = this->create_wall_timer(500ms, std::bind(&ssLocalPlanner::run, this));
    _publisher_path_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("path/markers", 10);

}

    // Publisher function for path - on a timer callback
void ssLocalPlanner::run()
{
  // Update _start with drone pose in map frame
  _start = {
    _lastPoseDrone.pose.pose.position.x,
    _lastPoseDrone.pose.pose.position.y,
    _lastPoseDrone.pose.pose.position.z
  };
  
  RCLCPP_INFO(this->get_logger(), "Start: %f %f %f"
  , _start[0], _start[1], _start[2]);
  auto path = Search::runSearch(*_costMap, _start, _goal);
  if (!path.has_value()){
    RCLCPP_WARN(this->get_logger(), "Search unable to find path to goal!");
  }
  else{
    visualizePath(path.value());
    Search::cleanPath(*_costMap, path.value());
    // Search::cleanPath(*_costMap, path.value());
  
    _lastPath.poses.clear();
    
    // compute point on _start voxel that intersects with first 
    double totalPathTime = 0.0;
    int i = 0;
    std::array<float,3> prevPoint = path.value().front();
    for(std::array<float,3> point : path.value())
    {
      auto pose = geometry_msgs::msg::PoseStamped();
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.position.z = point[2];
      pose.pose.orientation.x = 0.00081;
      pose.pose.orientation.y = 0.000069;
      pose.pose.orientation.z = 0.703855;
      pose.pose.orientation.w = 0.710343;
      if (i>0){
        // compute timing for path
        totalPathTime = totalPathTime + 1000 * sqrt(
          pow(pose.pose.position.x - prevPoint[0], 2) +
          pow(pose.pose.position.y - prevPoint[1], 2) +
          pow(pose.pose.position.z - prevPoint[2], 2));
        pose.header.stamp.sec = static_cast<int32_t>(totalPathTime);
      }
      else{
        // start time at 0 for first waypoint
        pose.header.stamp.sec = 0.0;
      }
  
      _lastPath.poses.push_back(pose);
      i++;
      prevPoint = point;
    }
    
    _lastPath.header.stamp = this->now();
    _lastPath.header.frame_id = "odom";
    _publisher->publish(_lastPath);
  }

}

void ssLocalPlanner::visualizePath(std::vector<std::array<float,3>>& path)
{
  visualization_msgs::msg::MarkerArray path_markers;
  int markerId = 0;
  for (std::array<float,3> pos : path)
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

void ssLocalPlanner::callback_drone(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  // std::lock_guard<std::mutex> lock(_mutex);
  _lastPoseDrone = *odometry; // odom frame
  // RCLCPP_INFO(this->get_logger(),"Drone Pose Received in odom frame: %f %f %f", 
  // _lastPoseDrone.pose.pose.position.x,
  // _lastPoseDrone.pose.pose.position.y,
  // _lastPoseDrone.pose.pose.position.z);
}

void ssLocalPlanner::callback_goal(const std_msgs::msg::Float32MultiArray::SharedPtr goal)
{
  // std::lock_guard<std::mutex> lock(_mutex);
  std::vector<float> goalData = goal->data;
  _goal = {goalData[0],goalData[1],goalData[2]};

  // RCLCPP_INFO(this->get_logger(),"Goal received from GlobalPlanner: %d %d %d", goalData[0],goalData[1],goalData[2]);
}
