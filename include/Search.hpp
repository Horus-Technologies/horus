#include "CostMap.hpp"
#include <unordered_map>
#include <queue>
#include <chrono>
#include <cmath>

using PathMap = std::unordered_map<std::array<int,3>,std::array<int,3>>;

namespace Search{
    std::unique_ptr<int[]> runBreadthFirst(const CostMap& costMap, const std::array<int,3>& start, const std::array<int,3>& goal);
    void cleanPath(const CostMap& costMap, std::vector<std::array<int,3>>& path);
}

// namespace std {
//     template <>
//     struct hash<std::array<int, 3>> {
//         size_t operator()(const std::array<int, 3>& arr) const {
//             size_t h = 0;
//             for (int val : arr) {
//                 h ^= std::hash<int>{}(val) + 0x9e3779b9 + (h << 6) + (h >> 2);
//             }
//             return h;
//         }
//     };
// }
