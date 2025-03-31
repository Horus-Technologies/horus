#include "Search.hpp"

namespace Search{
  PathMap runBreadthFirst(const CostMap& costMap, const std::array<int,3> start, const std::array<int,3> goal)
  {
    std::cout << "Breadth-first search starting" << std::endl;
    auto startTimer = std::chrono::high_resolution_clock::now();

    std::queue<std::array<int,3>> frontier;
    frontier.push(start);

    PathMap came_from;
    came_from[start] = start;

    int count = 0;
    while(!frontier.empty())
    {
      std::array<int,3> current = frontier.front();
      frontier.pop();

      if (current == goal)
      {
        break;
      }
      // std::cout << "current: " << current[0] << " " << current[1] << " " << current[2] << std::endl;
      for (std::array<int,3> next : costMap.emptyNeighbors(current))
      {
        // std::cout << "next: " << next[0] << " " << next[1] << " " << next[2] << std::endl;
        if (came_from.find(next) == came_from.end())
        {
          frontier.push(next);
          came_from[next] = current;
        }
      }
      count++;
      // "count = %d frontierSize = %ld",count, frontier.size());
    }
    // RCLCPP_INFO(this->get_logger(),
    //   "count = %d",count);
    auto endTimer = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = endTimer - startTimer;
    std::cout << "Breadth-first search finished in " << duration.count() << " sec" << std::endl;
    return came_from;
  }


  // Remove redundant waypoints from path
  void cleanPath(const CostMap& costMap, std::vector<std::array<int,3>>& path) // start --> goal
  { 
    std::cout << "Path cleaning started" << std::endl;
    
    auto startTimer = std::chrono::high_resolution_clock::now();
    int i = 0;
    std::array<int,3> current = path[0];
    bool foundGoal = false;
    if (path.size() <= 1) 
    {
      foundGoal = true;
      std::cout << "Path length (1) too short to clean." << std::endl;
    } 
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
        std::array<int,3> next = path[j];
        
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
      else
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
