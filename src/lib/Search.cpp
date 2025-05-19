#include "Search.hpp"

namespace Search{

  std::optional<std::vector<std::array<float,3>>> runSearch(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal){
    std::cout << "Search starting" << std::endl;
    auto startTimer = std::chrono::high_resolution_clock::now();

    if (costMap.getVoxelState(start) == VoxelState::OCCUPIED){
      // this means drone is in a wall and something is very wrong
      return std::nullopt;
    }

    std::optional<std::vector<std::array<float,3>>> path = std::nullopt;

    int localRegionSize = 16; // voxel count of local region cube side
    std::array<float,3> localGoal = findLocalGoal(costMap, start, goal, localRegionSize);
    // Check if local goal is obstructed by an obstacle
    while(!path.has_value()){
      if (costMap.getVoxelState(localGoal) == VoxelState::EMPTY){
        runBreadthFirst(costMap,start, localGoal, path, localRegionSize);
        if (path.has_value()){
          // viable path found
          cleanPath(costMap, path.value());
        }
      }
      else{ // current local goal is OCCUPIED
        // move localGoal 
        localGoal[0] = 
      }
    }
    

    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    std::cout << "Search finished in " << duration.count() << " sec" << std::endl;
    
    return path;
  }

  // Returns location within local region for goal - in world coordinates and in the local region frame
  std::array<float,3> findLocalGoal(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal, const int& localRegionSize){
    Eigen::Vector3f s(start[0],start[1],start[2]);
    Eigen::Vector3f g(goal[0],goal[1],goal[2]);
    // Eigen::Vector3f u = (g-s)/(g-s).norm(); // technically don't need to normalize..
    Eigen::Vector3f u = g-s;
    float u_max_component = std::max(u[0], std::max(u[1],u[2]));

    // Check if global goal is already inside the local region
    if (u_max_component < static_cast<float>(localRegionSize)/2){
      // simply shift to local region frame
      return {
        g[0] + (static_cast<float>(localRegionSize)/2), 
        g[1] + (static_cast<float>(localRegionSize)/2),
        g[2] + (static_cast<float>(localRegionSize)/2)
      };
    }
    else{
      // global goal is outside of the local region
  
      float t = (static_cast<float>(localRegionSize)/2) / u_max_component;
      t -= t / localRegionSize; // adjust to make sure the local goal is inside the local region
      // find local goal in the frame of local region
      std::array<float,3> local_goal = {
        u[0]*t + (static_cast<float>(localRegionSize)/2), 
        u[1]*t + (static_cast<float>(localRegionSize)/2),
        u[2]*t + (static_cast<float>(localRegionSize)/2)
      };
    }
  }

  void runBreadthFirst(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal,
  std::optional<std::vector<std::array<float,3>>>& path, const int& localRegionSize)
  {
    std::array<int,3> dims = {localRegionSize,localRegionSize,localRegionSize};

    CameFrom came_from(dims);
    
    // Make start_index in the CameFrom frame, which is a dense voxel grid of full map with all positive indices.
    // It should start in the middle of the local region
    std::array<int,3> start_index = {localRegionSize/2, localRegionSize/2, localRegionSize/2};

    // goal_index is also in the local region / CameFrom frame
    std::array<int,3> goal_index = costMap.worldToGlobal(goal);
    goal_index[0] = goal_index[0] - start_index[0] + localRegionSize/2;
    goal_index[1] = goal_index[1] - start_index[1] + localRegionSize/2;
    goal_index[2] = goal_index[2] - start_index[2] + localRegionSize/2;

    came_from.set(start_index,start_index);

    std::queue<std::array<int,3>> frontier;
    frontier.push(start_index);

    while(!frontier.empty())
    {
      std::array<int,3> current_index = frontier.front();
      frontier.pop();

      if (current_index == goal_index)
      {
        break;
      }
      // std::cout << "Current: " << current_index[0] << " " 
      // << current_index[1] << " "
      // << current_index[2] << std::endl;

      // Convert current_index to global index frame
      std::array<int,3> current_index_global;
      current_index_global[0] = current_index[0] + start_index[0] - localRegionSize/2;
      current_index_global[1] = current_index[1] + start_index[1] - localRegionSize/2;
      current_index_global[2] = current_index[2] + start_index[2] - localRegionSize/2;
      auto neighbors_optional = costMap.emptyNeighbors(current_index_global);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_index : neighbors_optional.value())
        {
          // Convert next_index to local region frame
          next_index[0] = next_index[0] - start_index[0] + localRegionSize/2;
          next_index[1] = next_index[1] - start_index[1] + localRegionSize/2;
          next_index[2] = next_index[2] - start_index[2] + localRegionSize/2;
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
    
    // Make sure goal was reached by BFS, otherwise set to nullopt and return.
    if (came_from.at(goal_index) == std::array<int,3>{-1,-1,-1}){
      path = std::nullopt;
      return;
    }
    // Obtain path
    path.value().clear(); //reset just in case path has waypoints in it already
    std::array<int,3> current_index = goal_index; // start out in local region frame
    while(current_index != start_index)
    {
      if (current_index == goal_index){
        std::cout << "safe here" << std::endl;
        path.value().push_back(goal);
        // std::cout << "safe here2" << std::endl;
      }
      else{
        // convert to global index frame
        std::array<int,3> current_index_global;
        current_index_global[0] = current_index[0] + start_index[0] - localRegionSize/2;
        current_index_global[1] = current_index[1] + start_index[1] - localRegionSize/2;
        current_index_global[2] = current_index[2] + start_index[2] - localRegionSize/2;
        std::array<float,3> current = costMap.globalToWorld(current_index_global);
        path.value().push_back(current);
      }
      // RCLCPP_INFO(this->get_logger(), "Current: %d %d %d"
      //   , current[0], current[1], current[2]);
      std::array<int,3> prev_index = came_from.at(current_index);
      current_index = prev_index;
    }
    path.value().push_back(start); // append start voxel
    
    std::reverse(path.value().begin(),path.value().end()); // start --> goal
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
