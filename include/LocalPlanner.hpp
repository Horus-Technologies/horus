/*
This file acts as the local path planner and 
generates local waypoints within local cost map.
*/

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>
#include "Search.hpp"


class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner(VoxelGrid* voxel_grid);

private:
    void run();
    void callback_drone(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void callback_goal(const std_msgs::msg::Float32MultiArray::SharedPtr goal);
    void visualize_path(std::vector<std::array<float,3>>& path);

    // Publisher members
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_raw;
    size_t _count;
    nav_msgs::msg::Path _last_path;

    // Subscriber members
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_drone;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _subscriber_goal;
    nav_msgs::msg::Odometry _last_pose_drone;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;

    VoxelGrid* _voxel_grid;
    std::array<float,3> _start;
    std::array<float,3> _goal;
};

#endif