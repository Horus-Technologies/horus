/*
Mapper node for voxelization of a point cloud from Realsense Camera
*/

#ifndef MAPPER_H
#define MAPPER_H

#include "CostMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class ssMapper : public rclcpp::Node
{
    public:
        ssMapper(CostMap* costMap);

    private:
        void run();
        void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
        void callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom);
        void processPoints();
        void visualizeCostMap();
        void transformBroadcast();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_pose;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _map_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _camera_broadcaster;

        CostMap* _costMap;
        sensor_msgs::msg::PointCloud2 _points;
        Eigen::Vector3f _position;
        Eigen::Vector3f _positionStart;
        Eigen::Quaternionf _orientation;
        int _count;
        bool _poseStartSet = false;
        bool _pointsReceived = false;
        std::array<float,3> _mapOffset;
        std::mutex _points_mutex;
};

#endif // MAPPER_H