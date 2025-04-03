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
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class ssMapper : public rclcpp::Node
{
    public:
        ssMapper(CostMap* costMap);

    private:
        void run();
        void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
        void callback_pose(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp);
        void processPoints();
        void visualizeCostMap();
        void transformBroadcast();

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriber_pose;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _map_broadcaster;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _camera_broadcaster;

        CostMap* _costMap;
        sensor_msgs::msg::PointCloud2 _points;
        Eigen::Vector3f _position;
        Eigen::Quaternionf _orientation;
        int _count;
        bool _pointsReceived = false;
        bool _poseReceived = false;
        std::array<float,3> _mapOffset;
        std::mutex _points_mutex;
};

#endif // MAPPER_H