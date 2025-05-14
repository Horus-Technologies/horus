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


class ssLocalPlanner : public rclcpp::Node
{
public:
    ssLocalPlanner(CostMap* costMap);

private:
    void run();
    void callback_drone(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void callback_goal(const std_msgs::msg::Float32MultiArray::SharedPtr goal);
    void visualizePath(std::vector<std::array<float,3>>& path);
    void visualizeCostMap();

    // Publisher members
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_path_markers;
    size_t _count;
    nav_msgs::msg::Path _lastPath;

    // Subscriber members
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriberDrone;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _subscriberGoal;
    nav_msgs::msg::Odometry _lastPoseDrone;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;

    CostMap* _costMap;
    std::array<float,3> _start;
    std::array<float,3> _goal;
    std::mutex _mutex;
};

#endif