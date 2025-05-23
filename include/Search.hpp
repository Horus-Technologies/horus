#include "VoxelGrid.hpp"
#include <unordered_map>
#include <queue>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

    std::optional<std::vector<std::array<float,3>>> run_search(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal);
    void run_breadth_first(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal,
        std::optional<std::vector<std::array<float,3>>>& path, const int& local_region_size);
    void run_a_star(const VoxelGrid& voxel_grid, const std::array<float,3>& start, const std::array<float,3>& goal,
        std::optional<std::vector<std::array<float,3>>>& path, const float& local_region_size);
    void clean_path(const VoxelGrid& voxel_grid, std::vector<std::array<float,3>>& path);
    float euclidean_distance(const std::array<float, 3>& a, const std::array<float, 3>& b);
}
