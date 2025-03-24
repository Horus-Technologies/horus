/*
This file acts as the local path planner and 
generates local waypoints within local cost map.
*/

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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "Search.hpp"

using namespace std::chrono_literals;

class ssLocalPlanner : public rclcpp::Node
{
  public:
    ssLocalPlanner()
    : Node("ssLocalPlanner"), _count(0), _costMap(0.25, {80,60,60}), _start(&_costMap.getVoxels()[0][0][0]), _goal(&_costMap.getVoxels()[79][10][10])
    {
        // Subscribing
        rclcpp::QoS qos(rclcpp::KeepLast(10)); 
        qos.best_effort();
        _subscriberDrone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp) {
            this->callback_drone(poseStamp);
        });
        
        // Publishing
        _publisher = this->create_publisher<nav_msgs::msg::Path>("waypoints", 10); // waypoints with stamped pose
        _timer = this->create_wall_timer(1000ms, std::bind(&ssLocalPlanner::run, this));
        _publisher_path_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("path/markers", 10);
        _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);

        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

        // Obstacle definition
        std::vector<std::array<double,3>> xyz_min;
        std::vector<std::array<double,3>> xyz_max;
        xyz_min.push_back({3, 3, 0});
        xyz_max.push_back({4, 5, 10});

        xyz_min.push_back({2, 0, 0});
        xyz_max.push_back({3, 3, 10});

        xyz_min.push_back({8, 4, 0});
        xyz_max.push_back({10, 8, 10});

        xyz_min.push_back({15, 0, 0});
        xyz_max.push_back({16, 4, 10});

        _obstacles[0] = xyz_min;
        _obstacles[1] = xyz_max;
    }

  private:
    // Publisher function for path - on a timer callback
    void run()
    {
      RCLCPP_INFO(this->get_logger(), "Finding drone start position");
      _start = _costMap.findVoxelByPosition({
        _lastPoseDrone.pose.position.x,
        _lastPoseDrone.pose.position.y,
        _lastPoseDrone.pose.position.z
      });

      // Check for new obstacles within drone range, and add them
      for (int i = 0; i < _obstacles[0].size();i++){
        std::array<double,3> xyz_min = _obstacles[0][i];
        std::array<double,3> xyz_max = _obstacles[1][i];
        float distanceToDrone = sqrt(
          pow(_lastPoseDrone.pose.position.x - (xyz_max[0] + xyz_min[0])/2, 2) +
          pow(_lastPoseDrone.pose.position.y - (xyz_max[1] + xyz_min[1])/2, 2) +
          pow(_lastPoseDrone.pose.position.z - (xyz_max[2] + xyz_min[2])/2, 2));
        RCLCPP_INFO(this->get_logger(), "Distance to Drone from Obstacle%d : %f", i, distanceToDrone);
        if(distanceToDrone < 5)
        {
          _costMap.addObstacle(_obstacles[0][i], _obstacles[1][i]);
        }
      }

      PathMap came_from = Search::runBreadthFirst(_costMap, _start, _goal);
      VoxelsRef voxels = _costMap.getVoxels();
      // Obtain path
      const Voxel* current = _goal;
      std::vector<const Voxel*> path;
      while(current != _start)
      {
        path.push_back(current);
        const Voxel* prev = came_from[current];
        current = prev;
      }
      path.push_back(_start); // append start voxel
      
      std::reverse(path.begin(),path.end()); // start --> goal
      visualizePath(path);
      Search::cleanPath(_costMap, path);
      Search::cleanPath(_costMap, path);
      visualizeCostMap();

      _lastPath.poses.clear();
      // add drone pose as the start of path
      // _lastPath.poses.push_back(_lastPoseDrone);
      
      // compute point on _start voxel that intersects with first 
      double totalPathTime = 0.0;
      int i = 0;
      const Voxel* prevVoxel = path.front();
      for(const Voxel* voxel : path)
      {
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.pose.position.x = voxel->getPosition()[0];
        pose.pose.position.y = voxel->getPosition()[1];
        pose.pose.position.z = voxel->getPosition()[2];
        pose.pose.orientation.x = 0.00081;
        pose.pose.orientation.y = 0.000069;
        pose.pose.orientation.z = 0.703855;
        pose.pose.orientation.w = 0.710343;
        if (i>0){
          totalPathTime = totalPathTime + 1000 * sqrt(
            pow(voxel->getPosition()[0] - prevVoxel->getPosition()[0], 2) +
            pow(voxel->getPosition()[1] - prevVoxel->getPosition()[1], 2) +
            pow(voxel->getPosition()[2] - prevVoxel->getPosition()[2], 2));
          pose.header.stamp.sec = static_cast<int32_t>(totalPathTime);
        }
        else{
          // i = 0
          pose.header.stamp.sec = 0.0;
        }

        _lastPath.poses.push_back(pose);
        i++;
        prevVoxel = voxel;
      }
      
      _lastPath.header.stamp = this->now();
      _lastPath.header.frame_id = "odom";
      _publisher->publish(_lastPath);
    }

  void visualizePath(std::vector<const Voxel*>& path)
  {
    visualization_msgs::msg::MarkerArray path_markers;
    int markerId = 0;
    for (const Voxel* voxel : path)
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
      std::array<double, 3> pos = voxel->getPosition();
      marker.pose.position.x = pos[0];
      marker.pose.position.y = pos[1];
      marker.pose.position.z = pos[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      // Set the scale
      const double scale = _costMap.getScale();
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

  void visualizeCostMap()
  {
    VoxelsRef voxels = _costMap.getVoxels();

    visualization_msgs::msg::MarkerArray marker_array;
    int markerId = 0;

    for (int i = 0; i < voxels.size(); ++i)
    {
      for (int j = 0; j < voxels[0].size(); ++j)
      {
        for (int k = 0; k < voxels[0][0].size(); ++k)
        {
          double cost = voxels[i][j][k].getCost();
          if (cost >= 2)
          {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // or your frame id
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "test_markers";
            marker.id = markerId;
            markerId++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose
            std::array<double, 3> pos = voxels[i][j][k].getPosition();
            marker.pose.position.x = pos[0];
            marker.pose.position.y = pos[1];
            marker.pose.position.z = pos[2];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            // Set the scale
            const double scale = _costMap.getScale();
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;

            // Set the color
            marker.color.r = 1.0f; // Red, decreasing with i
            marker.color.g = 0.0f;          // Green, increasing with i
            marker.color.b = 0.0f;
            marker.color.a = 0.9f;

            // Add the marker to the array
            marker_array.markers.push_back(marker);
          }
        }
      }
    }
    
    _publisher_map_markers->publish(marker_array);

  }

  void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp)
  {
    _lastPoseDrone = *poseStamp;
    // Print the received target pose message
    // RCLCPP_INFO(this->get_logger(), "Received Drone Pose:\nPosition: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f]",
    //             _lastPoseDrone.pose.position.x, _lastPoseDrone.pose.position.y, _lastPoseDrone.pose.position.z,
    //             _lastPoseDrone.pose.orientation.x, _lastPoseDrone.pose.orientation.y, _lastPoseDrone.pose.orientation.z, _lastPoseDrone.pose.orientation.w);
  }

  // Publisher members
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_path_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
  size_t _count;
  nav_msgs::msg::Path _lastPath;

  // Subscriber members
  // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _subscriber_target;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriberDrone;
  geometry_msgs::msg::PoseStamped _lastPoseTarget;
  geometry_msgs::msg::PoseStamped _lastPoseDrone;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;

  std::array<std::vector<std::array<double,3>>,2> _obstacles;
  CostMap _costMap;
  const Voxel* _start;
  const Voxel* _goal;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ssLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}