#include "Chunk.hpp"

Chunk::Chunk() : _res(16){
    _voxels.resize(_res*_res*_res, VoxelState::EMPTY);
}

Chunk::Chunk(int res) : _res(res){
    _voxels.resize(_res*_res*_res, VoxelState::EMPTY);
}

// flatten 3D grid
int Chunk::flatten(const std::array<int,3>& indices) const
{
    return indices[0] + _res*indices[1] + _res*_res*indices[2];
}

// unflatten 3D grid
std::array<int,3> Chunk::unflatten(int i) const
{
    return {i % _res, (i / _res) % _res, i / (_res*_res)}; 
}

VoxelState Chunk::getVoxelState(std::array<int,3> ind) const
{
    return _voxels[flatten(ind)];
}

void Chunk::setVoxelState(std::array<int,3> ind, VoxelState state)
{
    _voxels[flatten(ind)] = state;
}