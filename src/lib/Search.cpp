#include "Search.hpp"

namespace Search{

  std::optional<std::vector<std::array<float,3>>> runSearch(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal){
    std::cout << "Search starting" << std::endl;
    auto startTimer = std::chrono::high_resolution_clock::now();

    std::optional<std::vector<std::array<float,3>>> path;

    int localRegionSize = 16; // voxel count in each direction from start
    std::array<float,3> localGoal = findLocalGoal(costMap, start, goal, localRegionSize)
    runBreadthFirst(costMap,start, localGoal, path);

    cleanPath(costMap, path.value());

  }

  std::array<float,3> findLocalGoal(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal, const int& localRegionSize){
    Eigen::Vector3f s(start[0],start[1],start[2]);
    Eigen::Vector3f g(goal[0],goal[1],goal[2]);
    Eigen::Vector3f u = (g-s)/(g-s).norm(); // technically don't need to normalize

    Eigen::Vector3f minBound = s - Eigen::Vector3f::Constant(localRegionSize)/2;
    Eigen::Vector3f maxBound = s + Eigen::Vector3f::Constant(localRegionSize)/2;
    // Check if local goal is obstructed by an obstacle, in which case move out of it
    
  }

  void runBreadthFirst(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal,
  std::optional<std::vector<std::array<float,3>>>& path)
  {
    if (costMap.getVoxelState(start) == VoxelState::OCCUPIED){
      return std::nullopt;
    }

    auto limits = costMap.mapLimits(start,goal);
    std::array<float,3> minXYZ = limits.first;
    std::array<float,3> maxXYZ = limits.second;

    std::array<int,3> dims = {(maxXYZ[0] - minXYZ[0])/costMap.getScale(),
    (maxXYZ[1] - minXYZ[1])/costMap.getScale(),
    (maxXYZ[2] - minXYZ[2])/costMap.getScale()};

    CameFrom came_from(dims);
    
    // Make start_index in the CameFrom frame, which is a dense voxel grid of full map with all positive indices.
    std::array<int,3> start_index = costMap.worldToGlobal(start);
    start_index[0] += minXYZ[0];
    start_index[1] += minXYZ[1];
    start_index[2] += minXYZ[2];

    std::array<int,3> goal_index = costMap.worldToGlobal(goal);
    goal_index[0] += minXYZ[0];
    goal_index[1] += minXYZ[1];
    goal_index[2] += minXYZ[2];

    came_from.set(start_index,start_index);

    std::queue<std::array<int,3>> frontier;
    frontier.push(start_index);

    while(!frontier.empty())
    {
      std::array<int,3> current_index = frontier.front();
      frontier.pop();

      current_index[0] -= minXYZ[0];
      current_index[1] -= minXYZ[1];
      current_index[2] -= minXYZ[2];

      if (current_index == goal_index)
      {
        break;
      }
      // std::cout << "Current: " << current_index[0] << " " 
      // << current_index[1] << " "
      // << current_index[2] << std::endl;
      auto neighbors_optional = costMap.emptyNeighbors(current_index);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_index : neighbors_optional.value())
        {
          next_index[0] += minXYZ[0];
          next_index[1] += minXYZ[1];
          next_index[2] += minXYZ[2];
          // std::cout << "Next: " << next_index[0] << " " 
          // << next_index[1] << " "
          // << next_index[2] << std::endl;
        
          if (came_from.at(next_index) == std::array<int,3>{-1,-1,-1})
          {
            frontier.push(next_index);
            came_from.set(next_index, current_index);
          } 
        }
      }
    }

    if (came_from.at(goal_index) == std::array<int,3>{-1,-1,-1}){
      return std::nullopt;
    }

    // Obtain path
    std::array<int,3> current_index = goal_index;
    std::vector<std::array<float,3>> path;
    while(current_index != start_index)
    {
      if (current_index == goal_index){
        path.push_back(goal);
      }
      else{
        current_index[0] -= minXYZ[0];
        current_index[1] -= minXYZ[1];
        current_index[2] -= minXYZ[2];
        std::array<float,3> current = costMap.globalToWorld(current_index);
        path.push_back(current);
      }
      // RCLCPP_INFO(this->get_logger(), "Current: %d %d %d"
      //   , current[0], current[1], current[2]);
      std::array<int,3> prev_index = came_from.at(current_index);
      current_index = prev_index;
    }
    path.push_back(start); // append start voxel
    
    std::reverse(path.begin(),path.end()); // start --> goal

    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    std::cout << "Breadth-first search finished in " << duration.count() << " sec" << std::endl;
    return path;
  }


  // Remove redundant waypoints from path
  void cleanPath(const CostMap& costMap, std::vector<std::array<float,3>>& path) // start --> goal
  { 
    std::cout << "Path cleaning started" << std::endl;
    
    auto startTimer = std::chrono::high_resolution_clock::now();
    if (path.size() <= 1) 
    {
      std::cout << "Path length (1) too short to clean." << std::endl;
      return; //early exit
    } 
    int i = 0;
    std::array<float,3> current = path[0];
    std::cout << "Path cleaning: current obtained at start of path" << std::endl;
    bool foundGoal = false;
    while(!foundGoal) 
    {
      // check if there is a clear line of sight to the next voxel
      // continuously until a voxel is found where there is no clear line of sight.
      // then go back to the prev voxel and remove all of the ones between that
      // voxel and current
      bool foundFurthest = false; // flag indicating the furthest node within line of sight has been found
      int j = i+1;
      while (!foundFurthest)
      {
        std::array<float,3> next = path[j];
        
        if(next == path.back())
        {
          foundFurthest = true;
          if (costMap.checkCollision(current, next) == false)
          {
            foundGoal = true;
          }
          else
          {
            current = next;
          }
        }
        else if(costMap.checkCollision(current, next))
        {
          foundFurthest = true;
          current = next;
        }
        else
        {
          j++;
        }
      }
      //at this point j is the index of the voxel we would like to shortcut to

      //remove all voxels at indices between i and j
      if(foundGoal)
      {
        path.erase(path.begin()+i+1, path.begin()+j); //because want the final voxel in path to be the goal
      }
      else if(j - i != 1) //only erase if j had iterated at all
      {
        path.erase(path.begin()+i+1, path.begin()+j-1);
      }

      // after removal is done, move the index to the next voxel
      i++;
      current = path[i];      
    }
    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    std::cout << "Clean path finished in " << duration.count() << " sec" << std::endl;
  }
}
