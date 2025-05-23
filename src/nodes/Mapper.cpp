#include "Mapper.hpp"

using namespace std::chrono_literals;

Mapper::Mapper(VoxelGrid* voxel_grid)
: Node("Mapper"), _count(0), _voxel_grid(voxel_grid)
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
        500ms, std::bind(&Mapper::run, this));

    // Publishers
    _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);
}

void Mapper::run()
{
    if (_points_received && _pose_received){
        // transformBroadcast();
        process_points();
        visualize_grid();
    }
}

void Mapper::callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
    std::lock_guard<std::mutex> lock(_points_mutex);
    _points_buffer.push_front(points);

    if (_points_buffer.size() > 20){
        _points_buffer.pop_back();
    }
    if (!_points_received){
        _points_received = true;
        _points_start_time = rclcpp::Time(points->header.stamp).seconds();
    }

    // RCLCPP_INFO(this->get_logger(),"Points added to buffer. Buffer of size: %ld", _points_buffer.size());
}

void Mapper::callback_pose(const nav_msgs::msg::Odometry::SharedPtr odometry){
    std::lock_guard<std::mutex> lock(_points_mutex);
    _position[0] = odometry->pose.pose.position.x;
    _position[1] = odometry->pose.pose.position.y;
    _position[2] = odometry->pose.pose.position.z;

    _orientation.x() = odometry->pose.pose.orientation.x;
    _orientation.y() = odometry->pose.pose.orientation.y;
    _orientation.z() = odometry->pose.pose.orientation.z;
    _orientation.w() = odometry->pose.pose.orientation.w;

    if (!_pose_received){
        _pose_received = true;
        _pose_start_time = rclcpp::Time(odometry->header.stamp).seconds();
    }

    RCLCPP_INFO(this->get_logger(),"Drone Pose Received: %f %f %f", 
    _position[0],
    _position[1],
    _position[2]);

    double pose_sec = rclcpp::Time(odometry->header.stamp).seconds();

    RCLCPP_INFO(this->get_logger(),"Drone Pose Updated with Timestamp: %f", 
    pose_sec - _pose_start_time);

    // Match pose to points by timestamp and update _points
    find_best_points_match(rclcpp::Time(odometry->header.stamp));

    double point_sec = rclcpp::Time(_points.header.stamp).seconds();
    RCLCPP_INFO(this->get_logger(),"Point Updated with Timestamp: %f", 
    point_sec - _points_start_time);
}

void Mapper::find_best_points_match(rclcpp::Time poseTime){
    // Iterate through points buffer to and find the best match by timestamp
    sensor_msgs::msg::PointCloud2::SharedPtr best_point;
    double smallest_time_diff = std::numeric_limits<double>::max();
    for (const sensor_msgs::msg::PointCloud2::SharedPtr& point : _points_buffer)
    {
        rclcpp::Time point_time(point->header.stamp);
        double diff = std::abs((poseTime - point_time).seconds()) + _points_start_time - _pose_start_time;
        if(diff < smallest_time_diff){
            smallest_time_diff = diff;
            best_point = point;
        }
    }
    if (best_point) {
        _points = *best_point;
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid PointCloud2 found in buffer.");
        // handle this case appropriately
    }
}

void Mapper::process_points(){
    auto start_timer = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(_points_mutex);
    sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");
    Eigen::Matrix3f R = _orientation.toRotationMatrix();
    //loop through all points
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        //rotate points so they are aligned with robot orientation
        
        Eigen::Vector3f temp_point(*iter_x, *iter_y, *iter_z);
        temp_point = R*temp_point;

        // offset points so they are aligned with current robot position
        temp_point[0] += _position[0];
        temp_point[1] += _position[1];
        temp_point[2] += _position[2];

        _voxel_grid->set_voxel_state({temp_point[0], temp_point[1], temp_point[2]}, VoxelState::OCCUPIED);
            
        // inflate_recursively_from_index(indices, 0, 2);
    }

    auto end_timer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_timer - start_timer;
    RCLCPP_INFO(this->get_logger(),"Points processed in %f sec",duration.count());
}

// void Mapper::inflate_recursively_from_index(std::array<int,3> indices, int counter, int maxIterations)
// {
//     if (counter > maxIterations){
//         return;
//     }
//     std::vector<int> neighbors = _voxel_grid->emptyNeighbors(_voxel_grid->flatten(indices));
//     for (int i : neighbors){
//         _voxel_grid->setVoxelStateByIndices(_voxel_grid->unflatten(i), VoxelState::OCCUPIED);
//         inflate_recursively_from_index(_voxel_grid->unflatten(i), counter+1, maxIterations);
//     }
// }

void Mapper::visualize_grid()
{
    auto start_timer = std::chrono::high_resolution_clock::now();
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    _voxel_grid->for_each_voxel([&](float x, float y, float z) {
        // RCLCPP_INFO(this->get_logger(),"Voxel at (%f, %f, %f)",x,y,z);

        VoxelState state = _voxel_grid->get_voxel_state({x,y,z});
        if (state == VoxelState::OCCUPIED)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "markers";
            marker.id = marker_id;
            marker_id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            // Set the scale
            const double scale = _voxel_grid->get_scale();
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
    });

    _publisher_map_markers->publish(marker_array);
    auto end_timer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_timer - start_timer;
    RCLCPP_INFO(this->get_logger(),"Map markers publish finished in %f sec",duration.count());
}
    