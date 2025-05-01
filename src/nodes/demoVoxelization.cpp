/*
Demo node for voxelization of a point cloud.
*/

#include "CostMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class demoVoxelization : public rclcpp::Node
{
public:
demoVoxelization()
: Node("demoVoxelization"), _count(0), _costMap(0.1, {2,0,-0.25}), _positionStart({0,0,0}), _xOffset(2), _yOffset(0), _zOffset(-0.25)
    {   
        // Subscribers
        _subscriber_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            // "/lidar/points", 10, // when doing rosbag
            "/realsense/points", 10, // from realsense
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr points) {
                this->callback_points(points);
            });
        _subscriber_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            // "/ground_truth_pose", 10, // when doing rosbag
            "/odometry", 10, // from ardupilot
            [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
                this->callback_pose(odom);
            });

        // Timer
        _timer = this->create_wall_timer(
            100ms, std::bind(&demoVoxelization::run, this));

        // Publishers
        _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);
        _publisher_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_points", 10);
    }
private:
    void run()
    {
        if (_pointsReceived && _poseStartSet){

            processPoints();
            visualizeCostMap();
        }
    }

    void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
        _points = *points;
        _points.header.frame_id = "odom";
        if (!_pointsReceived){
            _pointsReceived = true;
        }
    }

    void callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom){
        _position[0] = odom->pose.pose.position.x;
        _position[1] = odom->pose.pose.position.y;
        _position[2] = odom->pose.pose.position.z;

        _orientation.x() = odom->pose.pose.orientation.x;
        _orientation.y() = odom->pose.pose.orientation.y;
        _orientation.z() = odom->pose.pose.orientation.z;
        _orientation.w() = odom->pose.pose.orientation.w;

        if (!_poseStartSet){
            _positionStart = _position;
            _poseStartSet = true;
        }
        // _position = _position - _positionStart; // for rosbag
    }

    void processPoints(){
        // RCLCPP_INFO(this->get_logger(), "Position: %f %f %f",
        // _position[0], _position[1], _position[2]);

        sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");

        //loop through all points
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            //rotate points so they are aligned with robot orientation
            Eigen::Matrix3f R = _orientation.toRotationMatrix();
            Eigen::Vector3f tempPoint(*iter_x, *iter_y, *iter_z);
            // Eigen::Matrix3f R_correction;
            // R_correction = Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitY());
            // Eigen::Vector3f rotatedPoint = R*R_correction*tempPoint; // for rosbag
            Eigen::Vector3f rotatedPoint = R*tempPoint;
            *iter_x = rotatedPoint[0];
            *iter_y = rotatedPoint[1];
            *iter_z = rotatedPoint[2];

            // offset points so they are aligned with current robot position
            *iter_x += _position[0] + _xOffset;
            //for rosbag
            // *iter_y += _position[1]+3;
            // *iter_z += _position[2]+1;
            // for realsense
            *iter_y += _position[1] + _yOffset;
            *iter_z += _position[2] + _zOffset;

            std::array<float,3> max_position = _costMap.getMaxPosition();

            //ensuring that point is not going to be outside of costmap
            if (*iter_x > 0 && *iter_x < max_position[0]
                && *iter_y > 0 && *iter_y < max_position[1]
                && *iter_z > 0 && *iter_z < max_position[2]){ 
                // determine which voxel in costmap this point belongs to
                _costMap.setVoxelStateByPosition({*iter_x, *iter_y, *iter_z}, VoxelState::OCCUPIED);
            }
        }

        _publisher_points->publish(_points);
    }

    void visualizeCostMap()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int markerId = 0;

    for (int i = 0; i < _costMap.getDims()[0]; ++i)
    {
      for (int j = 0; j < _costMap.getDims()[1]; ++j)
      {
        for (int k = 0; k < _costMap.getDims()[2]; ++k)
        {
          VoxelState state = _costMap.getVoxelStateByIndices({i,j,k});
          if (state == VoxelState::OCCUPIED)
          {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "markers";
            marker.id = markerId;
            markerId++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose
            std::array<float, 3> pos = _costMap.getVoxelPosition({i,j,k});
            marker.pose.position.x = pos[0] - _xOffset;
            marker.pose.position.y = pos[1] - _yOffset;
            marker.pose.position.z = pos[2] - _zOffset;
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
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;       
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            // Add the marker to the array
            marker_array.markers.push_back(marker);
          }
        }
      }
    }
    
    _publisher_map_markers->publish(marker_array);

  }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_pose;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_points;

    CostMap _costMap;
    sensor_msgs::msg::PointCloud2 _points;
    Eigen::Vector3f _position;
    Eigen::Vector3f _positionStart;
    Eigen::Quaternionf _orientation;
    int _count;
    bool _poseStartSet = false;
    bool _pointsReceived = false;
    float _xOffset;
    float _yOffset;
    float _zOffset; // mainly so that floor doesn't get filled up with voxels
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demoVoxelization>());
  rclcpp::shutdown();
  return 0;
}