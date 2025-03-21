#include "Voxel.hpp"
#include <vector>
#include <memory>
#include <unordered_set>
#include <iostream>

class CostMap
{
public:
    CostMap();
    CostMap(double scale, std::array<int,3> dims);
    const std::vector<std::vector<std::vector<Voxel>>>& getVoxels() const;
    const Voxel* getVoxel(std::array<int,3> index) const;
    const Voxel* findVoxelByPosition(std::array<double,3> position) const;
    double getScale(){ return _scale;};
    void addObstacle(std::array<double,3> xyz_min, std::array<double,3> xyz_max);
    const std::vector<const Voxel*> neighbors(const Voxel* voxel) const;
    bool checkCollision(const Voxel* voxelA, const Voxel* voxelB) const;

private:
    std::vector<std::vector<std::vector<Voxel>>> _voxels;
    const double _scale; // length of each voxel edge
    std::array<int,3> _dimensions;
};