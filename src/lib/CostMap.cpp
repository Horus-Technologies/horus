#include "CostMap.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>


CostMap::CostMap() : _scale(1.0), _voxels(){}

CostMap::CostMap(float scale) : _scale(scale){
    std::fill(_voxels.begin(), _voxels.end(), VoxelState::EMPTY);
}

// flatten 3D grid
int CostMap::flatten(const std::array<int,3>& indices) const
{
    return indices[0] + _res*indices[1] + _res*_res*indices[2];
}

// unflatten 3D grid
std::array<int,3> CostMap::unflatten(int i) const
{
    return {i % _res, (i / _res) % _res, i / (_res*_res)}; 
}

VoxelState CostMap::getVoxelStateByIndices(const std::array<int,3>& indices) const
{
    return _voxels[flatten(indices)];
}

std::array<float,3> CostMap::getVoxelPosition(const std::array<int,3>& indices) const
{
    return {(indices[0]+0.5)*_scale,(indices[1]+0.5)*_scale,(indices[2]+0.5)*_scale};
}

std::array<int,3> CostMap::getVoxelIndices(const std::array<float,3>& position) const{
    return {
        std::round(position[0]/_scale - 0.5),
        std::round(position[1]/_scale - 0.5),
        std::round(position[2]/_scale - 0.5)
    };
}

void CostMap::setVoxelStateByIndices(const std::array<int,3>& indices, const VoxelState& state)
{
    _voxels[flatten(indices)] = state;
}

void CostMap::setVoxelStateByPosition(const std::array<float,3>& position, const VoxelState& state)
{
    std::array<int,3> indices = {
        std::round(position[0]/_scale - 0.5),
        std::round(position[1]/_scale - 0.5),
        std::round(position[2]/_scale - 0.5)};
    // if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0)
    // {
    //     throw std::runtime_error("Voxel index found to be less than 0. Check CostMap limits.");
    // }
    setVoxelStateByIndices(indices, state);
}

void CostMap::addObstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max)
{   
    // compute xyz limits for obstacle that align with grid
    std::array<float,3> xyz_min_aligned;
    std::array<float,3> xyz_max_aligned;
    for (int i=0; i < 3; i++)
    {
        xyz_min_aligned[i] = std::floor(xyz_min[i]/_scale) * _scale;
        xyz_max_aligned[i] = std::ceil(xyz_max[i]/_scale) * _scale;
    }

    for (int j=0; j<_N; j++)
    {
        // get XYZ indices from flat array
        std::array<int,3> indices = unflatten(j);
        // get XYZ position from indices
        std::array<float,3> pos = {(indices[0]+0.5)*_scale,(indices[1]+0.5)*_scale,(indices[2]+0.5)*_scale};

        // adjust VoxelState if needed
        if (pos[0] >= xyz_min_aligned[0] && pos[0] <= xyz_max_aligned[0] &&
            pos[1] >= xyz_min_aligned[1] && pos[1] <= xyz_max_aligned[1] &&
            pos[2] >= xyz_min_aligned[2] && pos[2] <= xyz_max_aligned[2])
        {
            setVoxelStateByIndices(indices, VoxelState::OCCUPIED);
        }
    }
}

const std::vector<int> CostMap::emptyNeighbors(int index_flat) const
{
    std::vector<int> neighbors;
    std::array<int,3> indices = unflatten(index_flat);
    
    if (indices[0] < _res-1) // x axis +
    {
        if (_voxels[index_flat + 1] == VoxelState::EMPTY){
            neighbors.push_back(index_flat + 1);
        }
    }
    if (indices[0] > 0) // x axis -
    {
        if (_voxels[index_flat - 1] == VoxelState::EMPTY){
            neighbors.push_back(index_flat - 1);
        }
    }
    if (indices[1] < _res-1) // y axis +
    {
        if (_voxels[index_flat + _res] == VoxelState::EMPTY){
            neighbors.push_back(index_flat + _res);
        }
    }
    if (indices[1] > 0) // y axis -
    {
        if (_voxels[index_flat - _res] == VoxelState::EMPTY){
            neighbors.push_back(index_flat - _res);
        }
    }
    if (indices[2] < _res-1) // z axis +
    {
        if (_voxels[index_flat + _res*_res] == VoxelState::EMPTY){
            neighbors.push_back(index_flat + _res*_res);
        }
    }
    if (indices[2] > 0) // z axis -
    {
        if (_voxels[index_flat - _res*_res] == VoxelState::EMPTY){
            neighbors.push_back(index_flat - _res*_res);
        }
    }

    return neighbors;
}

// Amanatides & Woo Algorithm to traverse line of sight segment
bool CostMap::checkCollision(std::array<int,3>& voxelA, std::array<int,3>& voxelB) const
{
    // Initialization
    Eigen::Vector3f A(getVoxelPosition(voxelA)[0],getVoxelPosition(voxelA)[1],getVoxelPosition(voxelA)[2]);
    Eigen::Vector3f B(getVoxelPosition(voxelB)[0],getVoxelPosition(voxelB)[1],getVoxelPosition(voxelB)[2]);
    Eigen::Vector3f v = (B-A) / (B-A).norm(); // unit vector
    float X = voxelA[0];
    float Y = voxelA[1];
    float Z = voxelA[2];
    int stepX = (0 < v(0)) - (0 > v(0)); // 1 for positive x direction, -1 for negative, and 0 for neutral
    int stepY = (0 < v(1)) - (0 > v(1));
    int stepZ = (0 < v(2)) - (0 > v(2));
    float tMaxX;
    float tMaxY;
    float tMaxZ;
    if (v(0) == 0)
    {
        tMaxX = std::numeric_limits<float>::max();
    }
    else{
        tMaxX = (v * (_scale/2)/v(0)).norm(); // works only if ray starts from center of voxel
    }
    if (v(1) == 0)
    {
        tMaxY = std::numeric_limits<float>::max();
    }
    else{
        tMaxY = (v * (_scale/2)/v(1)).norm(); // works only if ray starts from center of voxel
    }
    if (v(2) == 0)
    {
        tMaxZ = std::numeric_limits<float>::max();
    }
    else{
        tMaxZ = (v * (_scale/2)/v(2)).norm(); // works only if ray starts from center of voxel
    }
    float tDeltaX = (v * (_scale)/v(0)).norm();
    float tDeltaY = (v * (_scale)/v(1)).norm();
    float tDeltaZ = (v * (_scale)/v(2)).norm();

    bool reachedVoxelB = false;
    int count = 0;
    // Incremental Traversal
    while(!reachedVoxelB)
    {
        if(tMaxX < tMaxY && tMaxX < tMaxZ) //tMaxX is smallest
        {   
            tMaxX = tMaxX + tDeltaX;
            X = X + stepX;
        }
        else if (tMaxY < tMaxZ) //tMaxY is smallest
        {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
        }
        else //tMaxZ is smallest
        {
            tMaxZ = tMaxZ + tDeltaZ;
            Z = Z + stepZ;
        }

        
        if (count == 1000)
        {
            throw std::runtime_error("Voxel traversal during collision check taking too long (count = 1000)");
        }

        if (getVoxelStateByIndices({X,Y,Z}) == VoxelState::OCCUPIED){
            // std::cout << "COLLISION FOUND" << std::endl;
            return true;
        }

        // check if voxelB has been reached
        if(X==voxelB[0] && Y==voxelB[1] && Z==voxelB[2]){
            reachedVoxelB = true;
        }
        count++;
    }


    // std::cout << "End collision check - no collision found!" << std::endl;
    return false;
}


std::array<float,3> CostMap::getMaxPosition() const
{
    return getVoxelPosition({_res-1,_res-1,_res-1});
}