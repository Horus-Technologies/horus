#ifndef COST_MAP_H
#define COST_MAP_H

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

class CostMap
{
public:
    CostMap();
    CostMap(float scale);
    std::array<int,3> worldToGlobal(const std::array<float,3>& position) const;
    std::array<float,3> globalToWorld(const std::array<int,3>& global_indices) const;
    std::pair<std::array<int,3>,std::array<int,3>> globalToLocal(const std::array<int,3>& global_indices) const;
    std::array<int,3> localToGlobal(const std::array<int,3>& chunk_indices,const std::array<int,3>& local_indices) const;
    VoxelState getVoxelState(const std::array<float,3>& position) const;
    void setVoxelState(std::array<float,3> position, VoxelState state);
    const std::optional<std::vector<std::array<int,3>>> emptyNeighbors(const std::array<int,3>& position) const;
    bool checkCollision(const std::array<float,3>& point1, const std::array<float,3>& point2) const;
    void addObstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max);
    void forEachVoxel(const std::function<void(float x, float y, float z)>& func);

    std::vector<std::array<int,3>> getChunkIndices() const;
    std::pair<std::array<float,3>, std::array<float,3>> mapLimits() const;

    float getScale() const { return _scale;} ;
    int getNumChunks() const {return _map.size();};

private:
    static constexpr int _res = 16;
    std::unordered_map<ChunkKey, Chunk> _map;
    const float _scale; // length of each voxel edge
};

#endif