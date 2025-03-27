#include "ssMapper.hpp"

using namespace std::chrono_literals;

ssMapper::ssMapper(CostMap* costMap)
: Node("ssMapper"), _count(0), _costMap(costMap), _positionStart({0,0,0}), _xOffset(1), _yOffset(0), _zOffset(-0.25)
{   
    // Subscribers
    _subscriber_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/realsense/points", 10, // from realsense
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr points) {
            this->callback_points(points);
        });
    _subscriber_pose = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10, // from ardupilot
        [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
            this->callback_pose(odom);
        });

    // Timer
    _timer = this->create_wall_timer(
        500ms, std::bind(&ssMapper::run, this));

    // Publishers
    _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);
    _publisher_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_points", 10);
}

void ssMapper::run()
{
    if (_pointsReceived && _poseStartSet){

        processPoints();
        visualizeCostMap();
    }
}

void ssMapper::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
    _points = *points;
    _points.header.frame_id = "odom";
    if (!_pointsReceived){
        _pointsReceived = true;
    }
}

void ssMapper::callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom){
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

void ssMapper::processPoints(){
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
        Eigen::Vector3f rotatedPoint = R*tempPoint;
        *iter_x = rotatedPoint[0];
        *iter_y = rotatedPoint[1];
        *iter_z = rotatedPoint[2];

        // offset points so they are aligned with current robot position
        *iter_x += _position[0] + _xOffset;
        *iter_y += _position[1] + _yOffset;
        *iter_z += _position[2] + _zOffset;

        std::array<double,3> max_position = _costMap->getDimensionsPosition();

        //ensuring that point is not going to be outside of costmap
        if (*iter_x > 0 && *iter_x < max_position[0]
            && *iter_y > 0 && *iter_y < max_position[1]
            && *iter_z > 0 && *iter_z < max_position[2]){ 
            // determine which voxel in costmap this point belongs to
            Voxel* voxel = _costMap->findVoxelByPosition({*iter_x, *iter_y, *iter_z});
            
            // increase cost by 3 or something of that voxel
            voxel->setCost(10);
        }
    }

    _publisher_points->publish(_points);
}

void ssMapper::visualizeCostMap()
{
    using VoxelsRef = const std::vector<std::vector<std::vector<Voxel>>>&;

    VoxelsRef voxels = _costMap->getVoxels();

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
                marker.pose.position.x = pos[0] - _xOffset;
                marker.pose.position.y = pos[1] - _yOffset;
                marker.pose.position.z = pos[2] - _zOffset;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                
                // Set the scale
                const double scale = _costMap->getScale();
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

    