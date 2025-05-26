#include "Search.hpp"

namespace Search{

  std::optional<std::vector<std::array<float,3>>> run_search(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal){
    std::cout << "Search starting" << std::endl;
    auto start_timer = std::chrono::high_resolution_clock::now();

    if (voxel_grid.get_voxel_state(start) == VoxelState::OCCUPIED){
      // this means drone is in a wall and something is very wrong
      return std::nullopt;
    }

    std::optional<std::vector<std::array<float,3>>> path = std::vector<std::array<float, 3>>{};

    int local_region_size = 32; // voxel count of local region cube side

    run_a_star(voxel_grid,start, goal, path, local_region_size);
    // if(path.has_value()){
    //   cleanPath(voxel_grid, path.value());
    // }

    auto end_timer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_timer - start_timer;
    std::cout << "Search finished in " << duration.count() << " sec" << std::endl;
    
    return path;
  }

  void run_breadth_first(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal,
  std::optional<std::vector<std::array<float,3>>>& path, const int& local_region_size)
  {
    std::array<int,3> dims = {local_region_size,local_region_size,local_region_size};

    CameFrom came_from(dims);
    
    // Make start_index in the CameFrom frame, which is a dense voxel grid of full map with all positive indices.
    // It should start in the middle of the local region
    std::array<int,3> start_local = {local_region_size/2, local_region_size/2, local_region_size/2};

    std::array<int,3> start_global = voxel_grid.world_to_global(start);

    std::array<int,3> goal_global = voxel_grid.world_to_global(goal);
    std::array<int,3> goal_local;
    goal_local[0] = goal_global[0] - (start_global[0] - local_region_size/2);
    goal_local[1] = goal_global[1] - (start_global[1] - local_region_size/2);
    goal_local[2] = goal_global[2] - (start_global[2] - local_region_size/2);

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

      // Convert current_index to global index frame
      std::array<int,3> current_global;
      current_global[0] = current_local[0] + (start_global[0] - local_region_size/2);
      current_global[1] = current_local[1] + (start_global[1] - local_region_size/2);
      current_global[2] = current_local[2] + (start_global[2] - local_region_size/2);
      auto neighbors_optional = voxel_grid.empty_neighbors(current_global);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_global : neighbors_optional.value())
        {
          // Convert next_index to local region frame
          std::array<int,3> next_local;
          next_local[0] = next_global[0] - (start_global[0] - local_region_size/2);
          next_local[1] = next_global[1] - (start_global[1] - local_region_size/2);
          next_local[2] = next_global[2] - (start_global[2] - local_region_size/2);
        
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
      if (count > local_region_size*local_region_size*local_region_size)
      {
        // goal was not found, must be located outside of local region
        path = std::nullopt;
        return;
      }

      if (current_local == goal_local){
        path.value().push_back(goal);
      }
      else{
        // convert to global index frame
        std::array<int,3> current_global;
        current_global[0] = current_local[0] + (start_global[0] - local_region_size/2);
        current_global[1] = current_local[1] + (start_global[1] - local_region_size/2);
        current_global[2] = current_local[2] + (start_global[2] - local_region_size/2);
        std::array<float,3> current = voxel_grid.global_to_world(current_global);
        path.value().push_back(current);
      }

      std::array<int,3> prev_index = came_from.at(current_local);
      current_local = prev_index;
      
      count++;
    }
    path.value().push_back(start); // append start voxel
    
    std::reverse(path.value().begin(),path.value().end()); // start --> goal
  }


  void run_a_star(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal,
  std::optional<std::vector<std::array<float,3>>>& path, const float& local_region_size)
  {
    std::array<int,3> dims = {local_region_size,local_region_size,local_region_size};

    CameFrom came_from(dims);
    
    // Make start_index in the CameFrom frame, which is a dense voxel grid of full map with all positive indices.
    // It should start in the middle of the local region
    std::array<int,3> start_local = {local_region_size/2, local_region_size/2, local_region_size/2};
    std::array<int,3> start_global = voxel_grid.world_to_global(start);

    std::array<int,3> goal_global = voxel_grid.world_to_global(goal);
    std::array<int,3> goal_local;
    goal_local[0] = goal_global[0] - (start_global[0] - local_region_size/2);
    goal_local[1] = goal_global[1] - (start_global[1] - local_region_size/2);
    goal_local[2] = goal_global[2] - (start_global[2] - local_region_size/2);

    came_from.set(start_local,start_local);

    typedef std::pair<float, std::array<int,3>> pq_elem;
    std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>> frontier;
    
    float start_cost = 0;
    pq_elem start_pq = {start_cost, start_local};
    frontier.push(start_pq);

    //keep track of which indices in local frame added last- for path reconstruction later
    std::array<int,3> last_local = start_local; 

    while(!frontier.empty())
    {
      auto current_pq = frontier.top();
      std::array<int,3> current_local = current_pq.second;
      frontier.pop();

      if (current_local == goal_local)
      {
        last_local = goal_local;
        break;
      }

      // Convert current_index to global index frame
      std::array<int,3> current_global;
      current_global[0] = current_local[0] + (start_global[0] - local_region_size/2);
      current_global[1] = current_local[1] + (start_global[1] - local_region_size/2);
      current_global[2] = current_local[2] + (start_global[2] - local_region_size/2);
      auto neighbors_optional = voxel_grid.empty_neighbors(current_global);
      if (neighbors_optional.has_value()){
        for (std::array<int,3> next_global : neighbors_optional.value())
        {
          std::array<float,3> next_world = voxel_grid.global_to_world(next_global);
          // Convert next_index to local region frame
          std::array<int,3> next_local;
          next_local[0] = next_global[0] - (start_global[0] - local_region_size/2);
          next_local[1] = next_global[1] - (start_global[1] - local_region_size/2);
          next_local[2] = next_global[2] - (start_global[2] - local_region_size/2);
  
          // check if this next voxel is outside of the local region, if it is, then simply break
          int max_next_local = *std::max_element(next_local.begin(), next_local.end());
          int min_next_local = *std::min_element(next_local.begin(), next_local.end());
          if (max_next_local >= local_region_size || min_next_local < 0){
            // A* has reached the local region wall and can end the search
            std::cout << "local region wall reached" << std::endl;
            frontier = std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>>();
            last_local = current_local;
            break;
          }
        
          // go only to unexplored empty neighbors
          if (came_from.at(next_local) == std::array<int,3>{-1,-1,-1})
          {
            float next_cost = euclidean_distance(start, next_world);
            float distance_estimate = euclidean_distance(goal, next_world);
            float next_estimated_cost = next_cost + distance_estimate;
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
    // std::cout << "last_local: "  << last_local[0] << ", " << last_local[1] << ", " << last_local[2] << std::endl;
    std::array<int,3> current_local = last_local; // start out in local region frame
    int count = 0;
    while(current_local != start_local)
    {
      if (current_local == goal_local){
        path.value().push_back(goal);
      }
      else{
        // convert to global index frame
        // std::cout << "current_local: "  << current_local[0] << ", " << current_local[1] << ", " << current_local[2] << std::endl;
        std::array<int,3> current_global;
        current_global[0] = current_local[0] + (start_global[0] - local_region_size/2);
        current_global[1] = current_local[1] + (start_global[1] - local_region_size/2);
        current_global[2] = current_local[2] + (start_global[2] - local_region_size/2);
        // std::cout << "current_global: "  << current_global[0] << ", " << current_global[1] << ", " << current_global[2] << std::endl;
        std::array<float,3> current = voxel_grid.global_to_world(current_global);
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
  void clean_path(const VoxelGrid& voxel_grid, std::vector<std::array<float,3>>& path) // start --> goal
  { 
    std::cout << "Path cleaning started" << std::endl;
    
    auto start_timer = std::chrono::high_resolution_clock::now();
    if (path.size() <= 1) 
    {
      std::cout << "Path length (1) too short to clean." << std::endl;
      return; //early exit
    } 
    int i = 0;
    std::array<float,3> current = path[0];
    std::cout << "Path cleaning: current obtained at start of path" << std::endl;
    bool found_goal = false;
    while(!found_goal) 
    {
      // check if there is a clear line of sight to the next voxel
      // continuously until a voxel is found where there is no clear line of sight.
      // then go back to the prev voxel and remove all of the ones between that
      // voxel and current
      bool found_furthest = false; // flag indicating the furthest node within line of sight has been found
      int j = i+1;
      while (!found_furthest)
      {
        std::array<float,3> next = path[j];
        
        if(next == path.back())
        {
          found_furthest = true;
          if (voxel_grid.check_collision(current, next) == false)
          {
            found_goal = true;
          }
          else
          {
            current = next;
          }
        }
        else if(voxel_grid.check_collision(current, next))
        {
          found_furthest = true;
          current = next;
        }
        else
        {
          j++;
        }
      }
      //at this point j is the index of the voxel we would like to shortcut to

      //remove all voxels at indices between i and j
      if(found_goal)
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
    auto end_timer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_timer - start_timer;
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
