#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include "Chunk.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>
#include <optional>
#include <utility>
// #include "chunkKeyHash.hpp"

using ChunkKey = std::array<int, 3>;

namespace std {
    template <>
    struct hash<ChunkKey> {
        std::size_t operator()(const ChunkKey& key) const {
            std::size_t h1 = std::hash<int>{}(key[0]);
            std::size_t h2 = std::hash<int>{}(key[1]);
            std::size_t h3 = std::hash<int>{}(key[2]);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

class VoxelGrid
{
public:
    VoxelGrid();
    VoxelGrid(float scale);
    std::array<int,3> world_to_global(const std::array<float,3>& position) const;
    std::array<float,3> global_to_world(const std::array<int,3>& global_indices) const;
    std::pair<std::array<int,3>,std::array<int,3>> global_to_local(const std::array<int,3>& global_indices) const;
    std::array<int,3> local_to_global(const std::array<int,3>& chunk_indices,const std::array<int,3>& local_indices) const;
    VoxelState get_voxel_state(const std::array<float,3>& position) const;
    void set_voxel_state(std::array<float,3> position, VoxelState state);
    const std::optional<std::vector<std::array<int,3>>> empty_neighbors(const std::array<int,3>& position) const;
    bool check_collision(const std::array<float,3>& point1, const std::array<float,3>& point2) const;
    void add_obstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max);
    void for_each_voxel(const std::function<void(float x, float y, float z)>& func);

    std::vector<std::array<int,3>> get_chunk_indices() const;
    std::pair<std::array<float,3>, std::array<float,3>> map_limits(const std::array<float,3>& start, const std::array<float,3>& goal) const;

    float get_scale() const { return _scale;} ;
    int get_num_chunks() const {return _map.size();};

private:
    static constexpr int _res = 16;
    std::unordered_map<ChunkKey, Chunk> _map;
    const float _scale; // length of each voxel edge
};

#endif