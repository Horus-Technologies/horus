#ifndef COST_MAP_H
#define COST_MAP_H


#include "Voxel.hpp"
#include <vector>
#include <memory>
#include <unordered_set>
#include <iostream>

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
    VoxelState getVoxelStateByIndices(std::array<int,3> indices);
    std::array<float,3> getVoxelPosition(std::array<int,3> indices);
    void setVoxelStateByIndices(std::array<int,3> indices, VoxelState state);
    void setVoxelStateByPosition(std::array<float,3> position, VoxelState state);
    void addObstacle(std::array<double,3> xyz_min, std::array<double,3> xyz_max);
    const std::vector<std::array<int,3>> emptyNeighbors(std::array<int,3> indices) const;
    bool checkCollision(std::array<int,3> voxelA, std::array<int,3> voxelB) const;
    std::array<float,3> getMaxPosition();
    float getScale(){ return _scale;};
    std::array<int,3> getDims(){ return {64,64,64};};


    // const std::array<VoxelState, N>& getVoxels() const;
    // Voxel* getVoxel(std::array<int,3> index);
    // Voxel* findVoxelByPosition(std::array<double,3> position);

private:
    constexpr int _N = 262144; // 64*64*64
    const float _scale; // length of each voxel edge
    std::array<VoxelState, _N> _voxels;
};

#endif