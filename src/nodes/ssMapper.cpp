#include "ssMapper.hpp"

using namespace std::chrono_literals;

ssMapper::ssMapper(CostMap* costMap)
: Node("ssMapper"), _count(0), _costMap(costMap), _mapOffset({0,0,0})
{   
    // Callback group
    auto exclusive_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = exclusive_group;

    // Subscribers
    _subscriber_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/realsense/points", 10, // from realsense
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr points) {
            this->callback_points(points);
        },options);

    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriber_pose = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr odometry) {
        this->callback_pose(odometry);
    },options);

    // Timer
    _timer = this->create_wall_timer(
        500ms, std::bind(&ssMapper::run, this));

    // Publishers
    _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);

    // Initialize the transform broadcaster
    _map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    _camera_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void ssMapper::run()
{
    if (_pointsReceived && _poseReceived){
        // transformBroadcast();
        processPoints();
        visualizeCostMap();
    }
}

void ssMapper::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
    std::lock_guard<std::mutex> lock(_points_mutex);
    _points_buffer.push_front(points);

    if (_points_buffer.size() > 20){
        _points_buffer.pop_back();
    }
    if (!_pointsReceived){
        _pointsReceived = true;
        _pointStartTime = rclcpp::Time(points->header.stamp).seconds();
    }
    double pointSec = rclcpp::Time(points->header.stamp).seconds();
    // RCLCPP_INFO(this->get_logger(),"Point Received with Timestamp: %f", 
    // pointSec);

    // RCLCPP_INFO(this->get_logger(),"Points added to buffer. Buffer of size: %ld", _points_buffer.size());
}

void ssMapper::callback_pose(const nav_msgs::msg::Odometry::SharedPtr odometry){
    std::lock_guard<std::mutex> lock(_points_mutex);
    _position[0] = odometry->pose.pose.position.x;
    _position[1] = odometry->pose.pose.position.y;
    _position[2] = odometry->pose.pose.position.z;

    _orientation.x() = odometry->pose.pose.orientation.x;
    _orientation.y() = odometry->pose.pose.orientation.y;
    _orientation.z() = odometry->pose.pose.orientation.z;
    _orientation.w() = odometry->pose.pose.orientation.w;

    if (!_poseReceived){
        _poseReceived = true;
        _poseStartTime = rclcpp::Time(odometry->header.stamp).seconds();
    }

    RCLCPP_INFO(this->get_logger(),"Drone Pose Received: %f %f %f", 
    _position[0],
    _position[1],
    _position[2]);

    double poseSec = rclcpp::Time(odometry->header.stamp).seconds();

    RCLCPP_INFO(this->get_logger(),"Drone Pose Updated with Timestamp: %f", 
    poseSec - _poseStartTime);

    // Match pose to points by timestamp and update _points
    findBestPointsMatch(rclcpp::Time(odometry->header.stamp));

    double pointSec = rclcpp::Time(_points.header.stamp).seconds();
    RCLCPP_INFO(this->get_logger(),"Point Updated with Timestamp: %f", 
    pointSec - _pointStartTime);
}

void ssMapper::findBestPointsMatch(rclcpp::Time poseTime){
    // Iterate through points buffer to and find the best match by timestamp
    sensor_msgs::msg::PointCloud2::SharedPtr bestPoint;
    double smallestTimeDiff = std::numeric_limits<double>::max();
    for (const sensor_msgs::msg::PointCloud2::SharedPtr& point : _points_buffer)
    {
        rclcpp::Time pointTime(point->header.stamp);
        double diff = std::abs((poseTime - pointTime).seconds()) + _pointStartTime - _poseStartTime;
        if(diff < smallestTimeDiff){
            smallestTimeDiff = diff;
            bestPoint = point;
        }
    }
    if (bestPoint) {
        _points = *bestPoint;
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid PointCloud2 found in buffer.");
        // handle this case appropriately
    }
}

void ssMapper::processPoints(){
    auto startTimer = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(_points_mutex);
    sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");
    Eigen::Matrix3f R = _orientation.toRotationMatrix();
    std::array<float,3> max_position = _costMap->getMaxPosition();
    //loop through all points
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        //rotate points so they are aligned with robot orientation
        
        Eigen::Vector3f tempPoint(*iter_x, *iter_y, *iter_z);
        tempPoint = R*tempPoint;

        // offset points so they are aligned with current robot position
        tempPoint[0] += _position[0] + _mapOffset[0];
        tempPoint[1] += _position[1] + _mapOffset[1];
        tempPoint[2] += _position[2] + _mapOffset[2];

        //ensuring that point is not going to be outside of costmap
        if (tempPoint[0] > 0 && tempPoint[0] < max_position[0]
            && tempPoint[1] > 0 && tempPoint[1] < max_position[1]
            && tempPoint[2] > 0 && tempPoint[2] < max_position[2]){ 
            // determine which voxel in costmap this point belongs to
            _costMap->setVoxelStateByPosition({tempPoint[0], tempPoint[1], tempPoint[2]}, VoxelState::OCCUPIED);
            // set neighbors to be occupied also
            std::array<int,3> indices = _costMap->getVoxelIndices({tempPoint[0], tempPoint[1], tempPoint[2]});
            std::vector<int> neighbors = _costMap->emptyNeighbors(_costMap->flatten(indices));
            for (int i : neighbors){
                _costMap->setVoxelStateByIndices(_costMap->unflatten(i), VoxelState::OCCUPIED);
            }
        }
    }

    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    RCLCPP_INFO(this->get_logger(),"Points processed in %f sec",duration.count());
}

// void ssMapper::inflateRecursively()
// {

// }

void ssMapper::visualizeCostMap()
{
    auto startTimer = std::chrono::high_resolution_clock::now();
    visualization_msgs::msg::MarkerArray marker_array;
    int markerId = 0;

    for (int i = 0; i < _costMap->getDims()[0]; ++i)
    {
        for (int j = 0; j < _costMap->getDims()[1]; ++j)
        {
        for (int k = 0; k < _costMap->getDims()[2]; ++k)
        {
            VoxelState state = _costMap->getVoxelStateByIndices({i,j,k});
            if (state == VoxelState::OCCUPIED)
            {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "markers";
            marker.id = markerId;
            markerId++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose
            std::array<float, 3> pos = _costMap->getVoxelPosition({i,j,k});
            marker.pose.position.x = pos[0];// - _xOffset;
            marker.pose.position.y = pos[1];// - _yOffset;
            marker.pose.position.z = pos[2];// - _zOffset;
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
    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    // RCLCPP_INFO(this->get_logger(),"Map markers publish finished in %f sec",duration.count());
}

void ssMapper::transformBroadcast()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "map";

    t.transform.translation.x = -_mapOffset[0];
    t.transform.translation.y = -_mapOffset[1];
    t.transform.translation.z = -_mapOffset[2];

    Eigen::Quaternionf q(1,0,0,0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    _map_broadcaster->sendTransform(t);

    // geometry_msgs::msg::TransformStamped t_camera;
    // t_camera.header.stamp = this->get_clock()->now();
    // t_camera.header.frame_id = "base_link";
    // t_camera.child_frame_id = "realsense_d435/link/realsense_d435";

    // t_camera.transform.translation.x = 0;
    // t_camera.transform.translation.y = 0;
    // t_camera.transform.translation.z = 0;
    // t_camera.transform.rotation.x = q.x();
    // t_camera.transform.rotation.y = q.y();
    // t_camera.transform.rotation.z = q.z();
    // t_camera.transform.rotation.w = q.w();

    // // Send the transformation
    // _camera_broadcaster->sendTransform(t_camera);
}
    