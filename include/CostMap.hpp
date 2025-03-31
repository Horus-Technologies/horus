#ifndef COST_MAP_H
#define COST_MAP_H


#include "Voxel.hpp"
#include <vector>
#include <memory>
#include <unordered_set>
#include <iostream>
#include <mutex>

enum VoxelState
{
    UNKNOWN,
    EMPTY,
    OCCUPIED
};

class CostMap
{
public:
    CostMap();
    CostMap(float scale);
    VoxelState getVoxelStateByIndices(std::array<int,3> indices) const;
    std::array<float,3> getVoxelPosition(std::array<int,3> indices) const;
    std::array<int,3> getVoxelIndices(std::array<float,3> position) const;
    void setVoxelStateByIndices(std::array<int,3> indices, VoxelState state);
    void setVoxelStateByPosition(std::array<float,3> position, VoxelState state);
    void addObstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max);
    const std::vector<std::array<int,3>> emptyNeighbors(std::array<int,3> indices) const;
    bool checkCollision(std::array<int,3> voxelA, std::array<int,3> voxelB) const;
    std::array<float,3> getMaxPosition() const;
    float getScale() const { return _scale;} ;
    std::array<int,3> getDims()const{ return {_res,_res,_res};};
    
private:
    int flatten(std::array<int,3> indices) const;
    std::array<int,3> unflatten(int i) const;

    static constexpr int _res = 100;
    static constexpr int _N = _res*_res*_res;
    const float _scale; // length of each voxel edge
    std::array<VoxelState, _N> _voxels;
    std::mutex map_mutex;
};

#endif