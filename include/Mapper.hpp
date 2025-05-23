/*
Mapper node for voxelization of a point cloud from Realsense Camera
*/

#ifndef MAPPER_H
#define MAPPER_H

#include "VoxelGrid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <deque>


using namespace std::chrono_literals;

class Mapper : public rclcpp::Node
{
    public:
        Mapper(VoxelGrid* voxel_grid);

    private:
        void run();
        void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
        void callback_pose(const nav_msgs::msg::Odometry::SharedPtr pose_stamp);
        void find_best_points_match(rclcpp::Time pose_time);
        void process_points();
        // void inflate_recursively_from_index(std::array<int,3> indices, int counter, int max_iterations);
        void visualize_grid();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_pose;

        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::TimerBase::SharedPtr _timer_broadcast;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;

        VoxelGrid* _voxel_grid;
        sensor_msgs::msg::PointCloud2 _points;
        Eigen::Vector3f _position;
        Eigen::Quaternionf _orientation;
        int _count;
        bool _points_received = false;
        bool _pose_received = false;
        std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> _points_buffer;
        double _pose_start_time;
        double _points_start_time;
        int _marker_id = 0;

        std::mutex _points_mutex;
};

#endif // MAPPER_H