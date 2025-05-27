
#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "VoxelGrid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstdlib>
#include <random>

class GlobalPlanner : public rclcpp::Node
{
    public:
        GlobalPlanner(VoxelGrid* voxel_grid);

    private:
        void run();
        void run_random();
        void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp);
        void visualize_path(std::vector<std::vector<float>>& path);

        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _publisher_goal;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_markers;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriber_drone;
        geometry_msgs::msg::PoseStamped _last_pose_drone;

        VoxelGrid* _voxel_grid;
        bool _plan_complete = false;
        std::vector<std::vector<float>> _goals;
        int _current_goal_index;
};

#endif // GLOBAL_PLANNER_H