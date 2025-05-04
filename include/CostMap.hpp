#ifndef COST_MAP_H
#define COST_MAP_H

#include "Chunk.hpp"
#include <vector>
#include <memory>
#include <unordered_map>
#include <iostream>
#include <mutex>

using ChunkKey = std::array<int, 3>;

class CostMap
{
public:
    CostMap();
    CostMap(float scale, std::array<float, 3> mapOffset);
    VoxelState getVoxelState(const std::array<float,3>& position) const;
    void setVoxelState(const std::array<float,3>& position, const VoxelState& state) const;
    void addObstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max);
    const std::vector<int> emptyNeighbors(int index_flat) const;
    bool checkCollision(const std::array<int,3>& voxelA, const std::array<int,3>& voxelB) const;

    // VoxelState getVoxelStateByIndices(const std::array<int,3>& indices) const;
    // std::array<float,3> getVoxelPosition(const std::array<int,3>& indices) const;
    // std::array<int,3> getVoxelIndices(const std::array<float,3>& position) const;
    // void setVoxelStateByIndices(const std::array<int,3>& indices, const VoxelState& state);
    // void setVoxelStateByPosition(const std::array<float,3>& position, const VoxelState& state);
    // std::array<float,3> getMaxPosition() const;
    float getScale() const { return _scale;} ;
    std::array<int,3> getDims()const{ return {_res,_res,_res};};
    std::array<float,3> getMapOffset() const{return _mapOffset;};

    
private:
    static constexp int _res = 16;
    std::unordered_map<ChunkKey, Chunk> _map;
    const float _scale; // length of each voxel edge
    std::array<float, 3> _mapOffset; // position offset from base (odom in our case) frame to costmap instance
};

#endif