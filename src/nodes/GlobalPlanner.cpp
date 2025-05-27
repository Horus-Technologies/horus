/*
Publishes a goal waypoint the drone must go to.
This node has access to the voxel_grid.
*/

#include "GlobalPlanner.hpp"

using namespace std::chrono_literals;

GlobalPlanner::GlobalPlanner(VoxelGrid* voxel_grid) 
: Node("GlobalPlanner"), _voxel_grid(voxel_grid), _current_goal_index(0)
{
    // Subscribing
    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriber_drone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ap/pose/filtered", qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp) {
        this->callback_drone(pose_stamp);
    });

    _publisher_goal = this->create_publisher<std_msgs::msg::Float32MultiArray>("/global_goal", 10);
    _publisher_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_plan/markers", 10);

    _timer = this->create_wall_timer(20ms, std::bind(&GlobalPlanner::run_random, this));

    _goals.push_back({1,20,6});
}

void GlobalPlanner::run()
{
    if(!_plan_complete){
        // Check if current goal is reached, and update _current_goal_index if so
        std::array<float,3> goal_point = {
            _goals[_current_goal_index][0],
            _goals[_current_goal_index][1],
            _goals[_current_goal_index][2]};
        
        float distance_to_goal_point = sqrt(
            pow(_last_pose_drone.pose.position.x - goal_point[0], 2) +
            pow(_last_pose_drone.pose.position.y - goal_point[1], 2) +
            pow(_last_pose_drone.pose.position.z - goal_point[2], 2));
        if (distance_to_goal_point < 0.05)
        {   
            _current_goal_index = _current_goal_index + 1;
            if(_current_goal_index == _goals.size())
            {
                // _planComplete = true;
                _current_goal_index = 0;
            }
        }
        std_msgs::msg::Float32MultiArray goal;
        std::vector<float> goal_data = _goals[_current_goal_index];
        goal.data = goal_data;
        _publisher_goal->publish(goal);
    }
}

void GlobalPlanner::run_random()
{
    std::array<float,3> goal_point = {_goals[0][0], _goals[0][1], _goals[0][2]};

    float distance_to_goal_point = sqrt(
        pow(_last_pose_drone.pose.position.x - goal_point[0], 2) +
        pow(_last_pose_drone.pose.position.y - goal_point[1], 2) +
        pow(_last_pose_drone.pose.position.z - goal_point[2], 2));
    if (distance_to_goal_point < 0.05 || _voxel_grid->get_voxel_state(goal_point) == VoxelState::OCCUPIED)
    {   
        std::mt19937 gen(std::random_device{}());
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        _goals[0] = {dist(gen)*20.0f, dist(gen)*20.0f, dist(gen)*8.0f};
        goal_point = {_goals[0][0], _goals[0][1], _goals[0][2]};
        while(_voxel_grid->get_voxel_state(goal_point) == VoxelState::OCCUPIED){
            _goals[0] = {dist(gen)*20.0f, dist(gen)*20.0f, dist(gen)*8.0f};
            goal_point = {_goals[0][0], _goals[0][1], _goals[0][2]};
        } 
    }
    std_msgs::msg::Float32MultiArray goal;
    std::vector<float> goal_data = _goals[_current_goal_index];
    goal.data = goal_data;
    _publisher_goal->publish(goal);
    // RCLCPP_INFO(this->get_logger(),"Goal sent: %f %f %f", 
    // _goals[0][0],
    // _goals[0][1],
    // _goals[0][2]);
    visualize_path(_goals);
}

void GlobalPlanner::callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp)
{
  _last_pose_drone = *pose_stamp; // odom frame
}

void GlobalPlanner::visualize_path(std::vector<std::vector<float>>& path)
{
  visualization_msgs::msg::MarkerArray path_markers;
  int marker_id = 0;
  for (std::vector<float> pos : path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "global_path";
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
    const float scale = 0.25;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Add the marker to the array
    path_markers.markers.push_back(marker);
  }
  _publisher_markers->publish(path_markers);
}