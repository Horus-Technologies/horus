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
            "/realsense/points", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr points) {
                this->callback_points(points);
            });
        _subscriber_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
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

        // Create a new linear transform with the desired voxel size
        openvdb::math::Transform::Ptr newTransform =
            openvdb::math::Transform::createLinearTransform(1.0f);

        // Set the grid's transform to the new transform
        _grid->setTransform(newTransform);
    
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
        _points.header.frame_id = "map";
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
        _position = _position - _positionStart;
    }

    void processPoints(){
        RCLCPP_INFO(this->get_logger(), "Processing");

        // Get the accessor for modifying voxel values
        openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

        // Set some voxels at specific coordinates
        accessor.setValue(openvdb::Coord(1, 0, 0), 1.0f);
        accessor.setValue(openvdb::Coord(2, 0, 0), 2.0f);
        accessor.setValue(openvdb::Coord(3, 0, 0), 3.0f);

        // Access and print a value
        // float value = accessor.getValue(openvdb::Coord(1, 2, 3));
        // RCLCPP_INFO(this->get_logger(),"Value at (1,2,3): %f", value);


        RCLCPP_INFO(this->get_logger(), "Position: %f %f %f",
        _position[0], _position[1], _position[2]);

        sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");

        //loop through all points
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            //rotate points so they are aligned with robot orientation
            Eigen::Matrix3f R = _orientation.toRotationMatrix();
            Eigen::Vector3f tempPoint(*iter_x, *iter_y, *iter_z);
            Eigen::Matrix3f R_correction;
            R_correction = Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitY());
            Eigen::Vector3f rotatedPoint = R*R_correction*tempPoint;
            *iter_x = rotatedPoint[0];
            *iter_y = rotatedPoint[1];
            *iter_z = rotatedPoint[2];

            // offset points so they are aligned with current robot position
            *iter_x += _position[0];
            *iter_y += _position[1]+ 3;
            *iter_z += _position[2]+1;

            openvdb::math::Vec3d worldPoint(*iter_x, *iter_y, *iter_z);
            openvdb::Coord ijk = _grid->transformPtr()->worldToIndexCellCentered(worldPoint);
            accessor.setValue(ijk, 1.0f);
            RCLCPP_INFO(this->get_logger(),"Added voxel at: %d %d %d ", ijk.x(), ijk.y(), ijk.z());
        }

        // _publisher_points->publish(_points);
    }

    void visualizeCostMap()
    {
        // Get the accessor for modifying voxel values
        // openvdb::FloatGrid::Accessor accessor = _grid->getAccessor();

        visualization_msgs::msg::MarkerArray marker_array;
        int markerId = 0;

        for (openvdb::FloatGrid::ValueOnCIter iter = _grid->cbeginValueOn(); iter; ++iter) {
            openvdb::Coord ijk = iter.getCoord();
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // or your frame id
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "test_markers";
            marker.id = markerId;
            markerId++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set the pose
            // Convert VDB index coordinates to world coordinates
            openvdb::Vec3d worldPos = _grid->indexToWorld(ijk);

            marker.pose.position.x = worldPos.x();
            marker.pose.position.y = worldPos.y();
            marker.pose.position.z = worldPos.z();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            // Set the scale
            openvdb::math::Transform::Ptr transform = _grid->transformPtr();
            openvdb::math::Vec3d voxelSize = transform->voxelSize(); 
            double scale = voxelSize.x();
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            
            // Set the color
            marker.color.r = 1.0f; // Red, decreasing with i
            marker.color.g = 0.0f;          // Green, increasing with i
            marker.color.b = 0.0f;
            marker.color.a = 0.6f;
            
            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }
        _publisher_map_markers->publish(marker_array);
    }
    
    

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_points;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber_pose;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_points;

    sensor_msgs::msg::PointCloud2 _points;
    bool _pointsReceived = false;
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