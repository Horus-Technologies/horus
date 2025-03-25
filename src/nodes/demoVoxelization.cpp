/*
Demo node for voxelization of a point cloud.
*/

#include "CostMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class demoVoxelization : public rclcpp::Node
{
public:
demoVoxelization() : Node("demoVoxelization"), _costMap(1, {40,40,40})
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
            1000ms, std::bind(&demoVoxelization::run, this));

        // Publishers
        _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);
    }
private:
    void run()
    {
        processPoints();
        visualizeCostMap();
    }

    void callback_points(const sensor_msgs::msg::PointCloud2::SharedPtr points){
        _points = points;
    }

    void callback_pose(const nav_msgs::msg::Odometry::SharedPtr odom){
        _pose = *odom.pose.pose;
    }

    void processPoints(){
        //loop through all points

            // determine xyz position of point 

            // determine which voxel in costmap this point belongs to

            // increase cost by 3 or something of that voxel

        // Looping through voxels. Might not be needed.
        // int x_dim = 40;
        // int y_dim = 40;
        // int z_dim = 40;

        // for (int i = 0; i < x_dim; ++i)
        // {
        //     for (int j = 0; j < y_dim; ++j)
        //     {
        //         for (int k = 0; k < z_dim; ++k)
        //         {
        //             if()
        //             double cost = 1;
        //             std::array<int,3> index = {i,j,k};
        //             Voxel voxel(index, cost, _scale);

        //             k_vec.push_back(voxel);
        //         }
        //     }
        // }
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
                marker.header.frame_id = "map"; // or your frame id
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
                marker.color.r = (1.0f - (cost * 0.2f)); // Red, decreasing with i
                marker.color.g = (cost * 0.2f);          // Green, increasing with i
                marker.color.b = 0.0f;
                marker.color.a = cost * 0.1f;

                // Add the marker to the array
                marker_array.markers.push_back(marker);
            }
            }
        }
        }
        
        _publisher_map_markers->publish(marker_array);

    }

    CostMap _costMap;
    sensor_msgs::msg::PointCloud2::SharedPtr _points;
    geometry_msgs::msg::Pose _pose;
    geometry_msgs::msg::Pose _poseStart;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demoVoxelization>());
  rclcpp::shutdown();
  return 0;
}