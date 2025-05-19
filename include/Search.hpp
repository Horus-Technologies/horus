#include "CostMap.hpp"
#include <unordered_map>
#include <queue>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using PathMap = std::unordered_map<std::array<int,3>,std::array<int,3>>;

namespace Search{

    struct CameFrom {
        std::unique_ptr<int[]> _data;
        std::array<int,3> _dims;

        CameFrom(const std::array<int,3>& dims) : _dims(dims){
            int arraySize = dims[0] * dims[1] * dims[2];
            _data = std::make_unique<int[]>(arraySize);
            for (size_t i = 1; i < arraySize; ++i) {
                _data[i] = -1;
            }
        }

        std::array<int,3> at(const std::array<int,3>& from){
        if (_data[flatten(from)] == -1)
        {
            return {-1,-1,-1};
        }
        else{
            return unflatten(_data[flatten(from)]);
        }
            
        }

        void set(const std::array<int,3>& from, const std::array<int,3>& to){
        _data[flatten(from)] = flatten(to);
        }

        int flatten(const std::array<int,3>& indices) const
        {
            return indices[0] + _dims[0]*indices[1] + _dims[0]*_dims[1]*indices[2];
        }

        std::array<int,3> unflatten(int i) const
        {
            return {i % _dims[0], (i / _dims[0]) % _dims[1], i / (_dims[0]*_dims[1])}; 
        }
    };

    std::optional<std::vector<std::array<float,3>>> runSearch(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal);
    std::array<float,3> findLocalGoal(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal, const int& localRegionSize);
    void runBreadthFirst(const CostMap& costMap, const std::array<float,3>& start, const std::array<float,3>& goal,
        std::optional<std::vector<std::array<float,3>>>& path, const int& localRegionSize);
    void cleanPath(const CostMap& costMap, std::vector<std::array<float,3>>& path);
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
