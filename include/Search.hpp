#include "CostMap.hpp"
#include <unordered_map>
#include <queue>
#include <chrono>

using PathMap = std::unordered_map<const Voxel*,const Voxel*>;
using VoxelsRef = const std::vector<std::vector<std::vector<Voxel>>>&;

namespace Search{
    PathMap runBreadthFirst(const CostMap& costMap, const Voxel* start, const Voxel* goal);
    void cleanPath(const CostMap& costMap, std::vector<const Voxel*>& path);
}
