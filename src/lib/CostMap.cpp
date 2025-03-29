#include "CostMap.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>


CostMap::CostMap() : _scale(1.0), _voxels(){}

CostMap::CostMap(double scale) : _scale(scale), _voxels(){}

// flatten 3D grid
int flatten(std::array<int,3> indices)
{
    return indices[0] + 64*indices[1] + 64*64*indices[2];
}

// unflatten 3D grid
std::array<int,3> indices unflatten(int i)
{
    return {i % 64, (i / 64) % 64, i / 64*64}; 
}

VoxelState CostMap::getVoxelStateByIndices(std::array<int,3> indices)
{
    return _voxels[flatten(indices)];
}

std::array<float,3> CostMap::getVoxelPosition(std::array<int,3> indices)
{
    return {(indices[0]+0.5)*_scale,(indices[1]+0.5)*_scale,(indices[2]+0.5)*_scale};
}

void CostMap::setVoxelStateByIndices(std::array<int,3> indices, VoxelState state)
{
    _voxels[flatten(indices)] = state;
}

void CostMap::setVoxelStateByPosition(std::array<float,3> position, VoxelState state)
{
    std::array<int,3> indices = {
        std::round(position[0]/_scale - 0.5),
        std::round(position[1]/_scale - 0.5),
        std::round(position[2]/_scale - 0.5)};
    if (indices[0] < 0 || indices[1] < 0 || indices[2] < 0)
    {
        throw std::runtime_error("Voxel index found to be less than 0. Check CostMap limits.");
    }
    setVoxelStateByIndices(indices, state);
}

std::array<float,3> CostMap::getMaxPosition()
{
    return getVoxelPosition({64,64,64});
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

const std::vector<std::array<int,3>> CostMap::emptyNeighbors(std::array<int,3> indices) const
{
    int index_flat = indices[0] + 64*indices[1] + 64*64*indices[2];
    std::vector<std::array<int,3>> neighbors;

    if (indices[0] < 64-1) // x axis +
    {
        if (_voxels[index_flat + 1] == VoxelState::EMPTY){
            neighbors.push_back(unflatten(index_flat + 1));
        }
    }
    if (index[0] > 0) // x axis -
    {
        if (_voxels[index_flat - 1] == VoxelState::EMPTY){
            neighbors.push_back(unflatten(index_flat - 1));
        }
    }
    if (index[1] < 64-1) // y axis +
    {
        const Voxel* next = &(_voxels[index[0]][index[1]+1][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[1] > 0) // y axis -
    {
        const Voxel* next = &(_voxels[index[0]][index[1]-1][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[2] < 64-1) // z axis +
    {
        const Voxel* next = &(_voxels[index[0]][index[1]][index[2]+1]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[2] > 0) // z axis -
    {
        const Voxel* next = &(_voxels[index[0]][index[1]][index[2]-1]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }

    return neighbors;
}

// Amanatides & Woo Algorithm to traverse line of sight segment
bool CostMap::checkCollision(const Voxel* voxelA, const Voxel* voxelB) const
{
    // Initialization
    Eigen::Vector3f A(voxelA->getPosition()[0],voxelA->getPosition()[1],voxelA->getPosition()[2]);
    Eigen::Vector3f B(voxelB->getPosition()[0],voxelB->getPosition()[1],voxelB->getPosition()[2]);
    Eigen::Vector3f v = (B-A) / (B-A).norm(); // unit vector
    float X = voxelA->getIndex()[0];
    float Y = voxelA->getIndex()[1];
    float Z = voxelA->getIndex()[2];
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
    const Voxel* current = voxelA;

    // Incremental Traversal
    while(current != voxelB)
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

        current = &_voxels[X][Y][Z];

        if (current == nullptr)
        {
            throw std::runtime_error("Current Voxel is Null");
        }

        if (current->getCost() > 2){
            // std::cout << "COLLISION FOUND" << std::endl;
            return true;
        }
    }


    // std::cout << "End collision check - no collision found!" << std::endl;
    return false;
}


std::array<float,3> CostMap::getMaxPosition()
{
    return getVoxel({_dimensions[0]-1,_dimensions[1]-1,_dimensions[2]-1})->getPosition();
}