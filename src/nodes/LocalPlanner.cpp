#include "LocalPlanner.hpp"

using namespace std::chrono_literals;

LocalPlanner::LocalPlanner(VoxelGrid* voxel_grid)
: Node("LocalPlanner"), _count(0), _voxel_grid(voxel_grid)
{

    // Callback group
    auto exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = exclusive_group;
    
    // Subscribing
    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriber_drone = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        this->callback_drone(odometry);
    },options);

    _subscriber_goal = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/global_goal", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr goal) {
          this->callback_goal(goal);
      },options);

    // Publishing
    _publisher = this->create_publisher<nav_msgs::msg::Path>("/local_plan/clean_path", 10); // waypoints with stamped pose
    _timer = this->create_wall_timer(500ms, std::bind(&LocalPlanner::run, this));
    _publisher_raw = this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_plan/raw_markers", 10);

}

// Publisher function for path - on a timer callback
void LocalPlanner::run()
{
  // Update _start with drone pose in map frame
  _start = {
    _last_pose_drone.pose.pose.position.x,
    _last_pose_drone.pose.pose.position.y,
    _last_pose_drone.pose.pose.position.z
  };
  
  RCLCPP_INFO(this->get_logger(), "Start: %f %f %f"
  , _start[0], _start[1], _start[2]);
  auto path = Search::run_search(*_voxel_grid, _start, _goal);
  if (!path.has_value()){
    RCLCPP_WARN(this->get_logger(), "Search unable to find path to goal!");
  }
  else{
    visualize_path(path.value());
    Search::clean_path(*_voxel_grid, path.value());
    Search::clean_path(*_voxel_grid, path.value());
    Search::clean_path(*_voxel_grid, path.value());
    Search::clean_path(*_voxel_grid, path.value());
    Search::clean_path(*_voxel_grid, path.value());
    Search::clean_path(*_voxel_grid, path.value()); // 2nd clean_path() is for certain edge cases
  
    _last_path.poses.clear();
    
    // compute point on _start voxel that intersects with first 
    double total_path_time = 0.0;
    int i = 0;
    std::array<float,3> prev_point = path.value().front();
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
        total_path_time = total_path_time + 1000 * sqrt(
          pow(pose.pose.position.x - prev_point[0], 2) +
          pow(pose.pose.position.y - prev_point[1], 2) +
          pow(pose.pose.position.z - prev_point[2], 2));
        pose.header.stamp.sec = static_cast<int32_t>(total_path_time);
      }
      else{
        // start time at 0 for first waypoint
        pose.header.stamp.sec = 0.0;
      }
  
      _last_path.poses.push_back(pose);
      i++;
      prev_point = point;
    }
    
    _last_path.header.stamp = this->now();
    _last_path.header.frame_id = "odom";
    _publisher->publish(_last_path);
  }
}

void LocalPlanner::visualize_path(std::vector<std::array<float,3>>& path)
{
  visualization_msgs::msg::MarkerArray path_markers;
  int marker_id = 0;
  for (std::array<float,3> pos : path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "local_raw_markers";
    marker.id = marker_id;
    marker_id++;
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
    const float scale = 0.125;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Add the marker to the array
    path_markers.markers.push_back(marker);
  }
  _publisher_raw->publish(path_markers);
}

void LocalPlanner::callback_drone(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  _last_pose_drone = *odometry; // odom frame
  // RCLCPP_INFO(this->get_logger(),"Drone Pose Received in odom frame: %f %f %f", 
  // _last_pose_drone.pose.pose.position.x,
  // _last_pose_drone.pose.pose.position.y,
  // _last_pose_drone.pose.pose.position.z);
}

void LocalPlanner::callback_goal(const std_msgs::msg::Float32MultiArray::SharedPtr goal)
{
  std::vector<float> goalData = goal->data;
  _goal = {goalData[0],goalData[1],goalData[2]};

  // RCLCPP_INFO(this->get_logger(),"Goal received from GlobalPlanner: %d %d %d", goalData[0],goalData[1],goalData[2]);
}
