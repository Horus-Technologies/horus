#include "CostMap.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <thread>


CostMap::CostMap()
    : _scale(1)
{

    int x_dim = 10;
    int y_dim = 10;
    int z_dim = 10;
    
    std::vector<std::vector<std::vector<Voxel>>> i_vec;
    for (int i = 0; i < x_dim; ++i)
    {
        std::vector<std::vector<Voxel>> j_vec;
        for (int j = 0; j < y_dim; ++j)
        {
            std::vector<Voxel> k_vec;
            for (int k = 0; k < z_dim; ++k)
            {
                
                double cost = 1;
                std::array<int,3> index = {i,j,k};
                Voxel voxel(index, cost, _scale);

                k_vec.push_back(voxel);
            }
            j_vec.push_back(k_vec);
        }
        i_vec.push_back(j_vec);
    }

    _voxels = i_vec;
}

CostMap::CostMap(double scale, std::array<int,3> dims)
    : _scale(scale), _dimensions(dims)
{
    int x_dim = _dimensions[0];
    int y_dim = _dimensions[1];
    int z_dim = _dimensions[2];

    std::vector<std::vector<std::vector<Voxel>>> i_vec;
    for (int i = 0; i < x_dim; ++i)
    {
        std::vector<std::vector<Voxel>> j_vec;
        for (int j = 0; j < y_dim; ++j)
        {
            std::vector<Voxel> k_vec;
            for (int k = 0; k < z_dim; ++k)
            {
                
                double cost = 1;
                std::array<int,3> index = {i,j,k};
                Voxel voxel(index, cost, _scale);

                k_vec.push_back(voxel);
            }
            j_vec.push_back(k_vec);
        }
        i_vec.push_back(j_vec);
    }

    _voxels = i_vec;
}

const std::vector<std::vector<std::vector<Voxel>>>& CostMap::getVoxels() const
{
    return _voxels;
}

const Voxel* CostMap::getVoxel(std::array<int,3> index) const
{
    if (index[0] < 0){index[0] = 0;}
    if (index[1] < 0){index[1] = 0;}
    if (index[2] < 0){index[2] = 0;}
    return &_voxels[index[0]][index[1]][index[2]];
}

const Voxel* CostMap::findVoxelByPosition(std::array<double,3> position) const
{
    std::array<int,3> index = {
        std::round(position[0]/_scale - 0.5),
        std::round(position[1]/_scale - 0.5),
        std::round(position[2]/_scale - 0.5)};
    return getVoxel(index);
}

/*
inputs: xyz_min, xyz_max

This function uses the inputs to define the position and size of the obstacle.
It then proceeds to modify the cost of all voxels that intersect with this obstacle
and sets them to 100, or some high value.

The resolution of the grid initially will affect how the obstacles appear, but this
setup will be conservative and will consider a Voxel occupied (high cost) if it crosses
the obstacle at all.
*/
void CostMap::addObstacle(std::array<double,3> xyz_min, std::array<double,3> xyz_max)
{   
    // compute xyz limits for obstacle that align with grid
    std::array<double,3> xyz_min_aligned;
    std::array<double,3> xyz_max_aligned;
    for (int i=0; i < 3; i++)
    {
        xyz_min_aligned[i] = std::floor(xyz_min[i]/_scale) * _scale;
        xyz_max_aligned[i] = std::ceil(xyz_max[i]/_scale) * _scale;
    }

    int x_dim = _dimensions[0];
    int y_dim = _dimensions[1];
    int z_dim = _dimensions[2];

    // iterate through all voxels in the grid
    for (int i = 0; i < x_dim; ++i)
    {
        for (int j = 0; j < y_dim; ++j)
        {
            for (int k = 0; k < z_dim; ++k)
            {
                std::array<double,3> pos = _voxels[i][j][k].getPosition();
                
                // check if the voxel crosses the obstacle
                if (pos[0] >= xyz_min_aligned[0] && pos[0] <= xyz_max_aligned[0] &&
                    pos[1] >= xyz_min_aligned[1] && pos[1] <= xyz_max_aligned[1] &&
                    pos[2] >= xyz_min_aligned[2] && pos[2] <= xyz_max_aligned[2])
                {
                    _voxels[i][j][k].setCost(12);
                }
            }
        }
    }
}

const std::vector<const Voxel*> CostMap::neighbors(const Voxel* voxel) const
{
    std::vector<const Voxel*> neighbors;
    std::array<int,3> index = voxel->getIndex();

    int x_dim = _dimensions[0];
    int y_dim = _dimensions[1];
    int z_dim = _dimensions[2];

    if (index[0] < x_dim-1)
    {
        const Voxel* next = &(_voxels[index[0]+1][index[1]][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[0] > 0)
    {
        const Voxel* next = &(_voxels[index[0]-1][index[1]][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[1] < y_dim-1)
    {
        const Voxel* next = &(_voxels[index[0]][index[1]+1][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[1] > 0)
    {
        const Voxel* next = &(_voxels[index[0]][index[1]-1][index[2]]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[2] < z_dim-1)
    {
        const Voxel* next = &(_voxels[index[0]][index[1]][index[2]+1]);
        if (next->getCost() < 10){
            neighbors.push_back(next);
        }
    }
    if (index[2] > 0)
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