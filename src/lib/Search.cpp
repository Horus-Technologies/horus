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

    runAStar(costMap,start, goal, path, localRegionSize);
    

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
    std::array<int,3> start_local = {localRegionSize/2, localRegionSize/2, localRegionSize/2};

    std::array<int,3> start_global = costMap.worldToGlobal(start);

    std::array<int,3> goal_global = costMap.worldToGlobal(goal);
    std::array<int,3> goal_local;
    goal_local[0] = goal_global[0] - (start_global[0] - localRegionSize/2);
    goal_local[1] = goal_global[1] - (start_global[1] - localRegionSize/2);
    goal_local[2] = goal_global[2] - (start_global[2] - localRegionSize/2);

    came_from.set(start_local,start_local);

    std::queue<std::array<int,3>> frontier;
    frontier.push(start_local);

    while(!frontier.empty())
    {
      std::array<int,3> current_local = frontier.front();
      frontier.pop();

      if (current_local == goal_local)
      {
        break;
      }
      // std::cout << "Current: " << current_index[0] << " " 
      // << current_index[1] << " "
      // << current_index[2] << std::endl;

      // Convert current_index to global index frame
      std::array<int,3> current_global;
      current_global[0] = current_local[0] + (start_global[0] - localRegionSize/2);
      current_global[1] = current_local[1] + (start_global[1] - localRegionSize/2);
      current_global[2] = current_local[2] + (start_global[2] - localRegionSize/2);
      auto neighbors_optional = costMap.emptyNeighbors(current_global);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_global : neighbors_optional.value())
        {
          // Convert next_index to local region frame
          std::array<int,3> next_local;
          next_local[0] = next_global[0] - (start_global[0] - localRegionSize/2);
          next_local[1] = next_global[1] - (start_global[1] - localRegionSize/2);
          next_local[2] = next_global[2] - (start_global[2] - localRegionSize/2);
          // std::cout << "Next: " << next_index[0] << " " 
          // << next_index[1] << " "
          // << next_index[2] << std::endl;
        
          if (came_from.at(next_local) == std::array<int,3>{-1,-1,-1})
          {
            frontier.push(next_local);
            came_from.set(next_local, current_local);
          } 
        }
      }
    }
    
    // Make sure goal was reached by BFS, otherwise set to nullopt and return.
    if (came_from.at(goal_local) == std::array<int,3>{-1,-1,-1}){
      path = std::nullopt;
      return;
    }
    // Obtain path
    path.value().clear(); //reset just in case path has waypoints in it already
    std::array<int,3> current_local = goal_local; // start out in local region frame
    int count = 0;
    while(current_local != start_local)
    {
      if (count > localRegionSize*localRegionSize*localRegionSize)
      {
        // goal was not found, must be located outside of local region
        path = std::nullopt;
        return;
      }

      if (current_local == goal_local){
        std::cout << "safe here" << std::endl;
        path.value().push_back(goal);
        // std::cout << "safe here2" << std::endl;
      }
      else{
        // convert to global index frame
        std::array<int,3> current_global;
        current_global[0] = current_local[0] + (start_global[0] - localRegionSize/2);
        current_global[1] = current_local[1] + (start_global[1] - localRegionSize/2);
        current_global[2] = current_local[2] + (start_global[2] - localRegionSize/2);
        std::array<float,3> current = costMap.globalToWorld(current_global);
        path.value().push_back(current);
      }
      // RCLCPP_INFO(this->get_logger(), "Current: %d %d %d"
      //   , current[0], current[1], current[2]);
      std::array<int,3> prev_index = came_from.at(current_local);
      current_local = prev_index;
      
      count++;
    }
    path.value().push_back(start); // append start voxel
    
    std::reverse(path.value().begin(),path.value().end()); // start --> goal
  }


  void runAStar(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal,
  std::optional<std::vector<std::array<float,3>>>& path, const float& localRegionSize)
  {
    std::array<int,3> dims = {localRegionSize,localRegionSize,localRegionSize};

    CameFrom came_from(dims);
    CostSoFar cost_so_far(dims);
    
    // Make start_index in the CameFrom frame, which is a dense voxel grid of full map with all positive indices.
    // It should start in the middle of the local region
    std::array<int,3> start_local = {localRegionSize/2, localRegionSize/2, localRegionSize/2};
    std::array<int,3> start_global = costMap.worldToGlobal(start);

    std::array<int,3> goal_global = costMap.worldToGlobal(goal);
    std::array<int,3> goal_local;
    goal_local[0] = goal_global[0] - (start_global[0] - localRegionSize/2);
    goal_local[1] = goal_global[1] - (start_global[1] - localRegionSize/2);
    goal_local[2] = goal_global[2] - (start_global[2] - localRegionSize/2);

    came_from.set(start_local,start_local);
    cost_so_far.set(start_local, 0);

    typedef std::pair<float, std::array<int,3>> pq_elem;
    std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>> frontier;
    
    float start_cost = 0;
    pq_elem start_pq = {start_cost, start_local};
    frontier.push(start_pq);

    //keep track of which indices in local frame added last- for path reconstruction later
    std::array<int,3> last_local = start_local; 
    std::cout << "goal_local: " << goal_local[0] << " " 
      << goal_local[1] << " "
      << goal_local[2] << std::endl;

    while(!frontier.empty())
    {
      auto current_pq = frontier.top();
      std::array<int,3> current_local = current_pq.second;
      frontier.pop();

      std::cout << "current_local: " << current_local[0] << " " 
      << current_local[1] << " "
      << current_local[2] << std::endl;

      if (current_local == goal_local)
      {
        last_local = goal_local;
        break;
      }


      // Convert current_index to global index frame
      std::array<int,3> current_global;
      current_global[0] = current_local[0] + (start_global[0] - localRegionSize/2);
      current_global[1] = current_local[1] + (start_global[1] - localRegionSize/2);
      current_global[2] = current_local[2] + (start_global[2] - localRegionSize/2);
      auto neighbors_optional = costMap.emptyNeighbors(current_global);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_global : neighbors_optional.value())
        {
          std::array<float,3> next_world = costMap.globalToWorld(next_global);
          // Convert next_index to local region frame
          std::array<int,3> next_local;
          next_local[0] = next_global[0] - (start_global[0] - localRegionSize/2);
          next_local[1] = next_global[1] - (start_global[1] - localRegionSize/2);
          next_local[2] = next_global[2] - (start_global[2] - localRegionSize/2);
          

          // std::array distanceFromLocalWall = {next_local}
          // check if this next voxel is outside of the local region, if it is, then simply break
          int max_next_local = *std::max_element(next_local.begin(), next_local.end());
          int min_next_local = *std::min_element(next_local.begin(), next_local.end());
          if (max_next_local >= localRegionSize || min_next_local < 0){
            // A* has reached the local region wall and can end the search
            std::cout << "local region wall reached" << std::endl;
            frontier = std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>>();
            last_local = current_local;
            break;
          }
        
          // go only to unexplored empty neighbors
          if (came_from.at(next_local) == std::array<int,3>{-1,-1,-1})
          {
            // std::cout << "next_local: " << next_local[0] << " " 
            // << next_local[1] << " "
            // << next_local[2] << std::endl;
            // float next_cost = cost_so_far.at(current_local) + 1;
            float next_cost = euclidean_distance(start, next_world);
            // cost_so_far.set(next_local, next_cost);
            float distance_estimate = euclidean_distance(goal, next_world);
            float next_estimated_cost = next_cost + distance_estimate;
            // std::cout << "next_estimated_cost " << next_estimated_cost << std::endl;
            pq_elem next_pq = {next_estimated_cost, next_local};
            frontier.push(next_pq);
            came_from.set(next_local, current_local);
            last_local = next_local;
          } 
        }
      }
    }
    
    // Obtain path
    path.value().clear(); //reset just in case path has waypoints in it already
    std::cout << "last_local: "  << last_local[0] << ", " << last_local[1] << ", " << last_local[2] << std::endl;
    std::array<int,3> current_local = last_local; // start out in local region frame
    int count = 0;
    while(current_local != start_local)
    {
      if (current_local == goal_local){
        path.value().push_back(goal);
      }
      else{
        // convert to global index frame
        std::cout << "current_local: "  << current_local[0] << ", " << current_local[1] << ", " << current_local[2] << std::endl;
        std::array<int,3> current_global;
        current_global[0] = current_local[0] + (start_global[0] - localRegionSize/2);
        current_global[1] = current_local[1] + (start_global[1] - localRegionSize/2);
        current_global[2] = current_local[2] + (start_global[2] - localRegionSize/2);
        std::cout << "current_global: "  << current_global[0] << ", " << current_global[1] << ", " << current_global[2] << std::endl;
        std::array<float,3> current = costMap.globalToWorld(current_global);
        path.value().push_back(current);
      }
      // RCLCPP_INFO(this->get_logger(), "Current: %d %d %d"
      //   , current[0], current[1], current[2]);
      std::array<int,3> prev_index = came_from.at(current_local);
      current_local = prev_index;

      count++;
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

  float euclidean_distance(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    float sum = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}
}
