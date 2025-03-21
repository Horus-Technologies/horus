/*
Demo node for breadth-first search on a voxel cost map grid.
*/

#include "Search.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class demoBFS : public rclcpp::Node
{
public:
    demoBFS() : Node("demoBFS"), _costMap(1, {40,40,40}), _start(&_costMap.getVoxels()[0][5][0]), _goal(&_costMap.getVoxels()[35][10][0])
    {
        // Timer
        _timer = this->create_wall_timer(
            1000ms, std::bind(&demoBFS::run, this));

        // Publisher for Rviz visualization
        _publisher_path_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("path/markers", 10);
        _publisher_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("map/markers", 10);

        buildCostMap();
    }
private:
    void run()
    {
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
        Search::cleanPath(_costMap, path);
        visualizePath(path);
        visualizeCostMap();
    }

    void buildCostMap()
    {
        std::vector<std::array<double,3>> xyz_min;
        std::vector<std::array<double,3>> xyz_max;
        xyz_min.push_back({3, 3, 0});
        xyz_max.push_back({4, 5, 3});

        xyz_min.push_back({2, 0, 0});
        xyz_max.push_back({3, 3, 3});

        xyz_min.push_back({2, 9, 0});
        xyz_max.push_back({6, 12, 6});

        xyz_min.push_back({8, 0, 0});
        xyz_max.push_back({30, 10, 5});

        for (int i = 0; i < xyz_min.size();i++){
        _costMap.addObstacle(xyz_min[i], xyz_max[i]);
        }
    }
    void visualizePath(std::vector<const Voxel*>& path)
  {
    visualization_msgs::msg::MarkerArray path_markers;
    int markerId = 0;
    for (const Voxel* voxel : path)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
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

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_path_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;

  CostMap _costMap;
  const Voxel* _start;
  const Voxel* _goal;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demoBFS>());
  rclcpp::shutdown();
  return 0;
}