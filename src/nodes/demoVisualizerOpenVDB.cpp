/*
Demo node for visualization of OpenVDB map.
*/

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <openvdb/openvdb.h>

using namespace std::chrono_literals;

class demoVisualizerOpenVDB : public rclcpp::Node
{
public:
demoVisualizerOpenVDB()
: Node("demoVisualizerOpenVDB"), _count(0), _positionStart({0,0,0})
    {   
        // Subscribers
        _subscriber_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr points) {
                this->callback_points(points);
            });
        _subscriber_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth_pose", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
                this->callback_pose(odom);
            });

        // Timer
        _timer = this->create_wall_timer(
            100ms, std::bind(&demoVisualizerOpenVDB::run, this));

        // Publishers
        _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);
        _publisher_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_points", 10);
            
        openvdb::initialize();

        // Create a floating-point grid with a background value of 0
        _grid = openvdb::FloatGrid::create(/*background value=*/0.0);

        // Set a name for the grid (optional)
        _grid->setName("MyVoxelGrid");
    
    }
private:
    void run()
    {
        processPoints();
    }

    void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
        _points = *points;
        _points.header.frame_id = "map";
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
        _position = _position - _positionStart;
    }

    void processPoints(){
        // RCLCPP_INFO(this->get_logger(), "Position: %f %f %f",
        // _position[0], _position[1], _position[2]);

        // Get the accessor for modifying voxel values
        openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

        // Set some voxels at specific coordinates
        accessor.setValue(openvdb::Coord(1, 2, 3), 1.0f);
        accessor.setValue(openvdb::Coord(2, 2, 3), 2.0f);
        accessor.setValue(openvdb::Coord(3, 2, 3), 3.0f);

        // Access and print a value
        float value = accessor.getValue(openvdb::Coord(1, 2, 3));
        RCLCPP_INFO(this->get_logger(),"Value at (1,2,3): %f", value);
    }

    

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_pose;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_points;

    sensor_msgs::msg::PointCloud2 _points;
    Eigen::Vector3f _position;
    Eigen::Vector3f _positionStart;
    Eigen::Quaternionf _orientation;
    int _count;
    bool _poseStartSet = false;

    openvdb::FloatGrid::Ptr _grid;
    // openvdb::FloatGrid::Accessor _accessor;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demoVisualizerOpenVDB>());
  rclcpp::shutdown();
  return 0;
}