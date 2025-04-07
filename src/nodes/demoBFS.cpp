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
    demoBFS() : Node("demoBFS"), _costMap(1,{0,0,0}), _start({0,0,0}), _goal({40,0,10})
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
        std::unique_ptr<int[]> came_from = Search::runBreadthFirst(_costMap, _start, _goal);
    
        // // Obtain path
        std::array<int,3> current = _goal;
        std::vector<std::array<int,3>> path;
        while(current != _start)
        {
          path.push_back(current);
          std::array<int,3> prev = _costMap.unflatten(
            came_from[_costMap.flatten(current)]);
          // RCLCPP_INFO(this->get_logger(), "prev: %d %d %d", prev[0],prev[1],prev[2]);
          current = prev;
        }
        path.push_back(_start); // append start voxel
        
        std::reverse(path.begin(),path.end()); // start --> goal
        visualizePath(path);
        Search::cleanPath(_costMap, path);
        visualizeCostMap();
    }

    void buildCostMap()
    {
        std::vector<std::array<float,3>> xyz_min;
        std::vector<std::array<float,3>> xyz_max;
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
    void visualizePath(std::vector<std::array<int,3>>& path)
  {
    visualization_msgs::msg::MarkerArray path_markers;
    int markerId = 0;
    for (std::array<int,3> indices : path)
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
      std::array<float, 3> pos = _costMap.getVoxelPosition(indices);
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
            marker.header.frame_id = "map"; // or your frame id
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "test_markers";
            marker.id = markerId;
            markerId++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose
            std::array<float, 3> pos = _costMap.getVoxelPosition({i,j,k});
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

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_path_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _publisher_map_markers;

  CostMap _costMap;
  std::array<int,3> _start;
  std::array<int,3> _goal;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demoBFS>());
  rclcpp::shutdown();
  return 0;
}