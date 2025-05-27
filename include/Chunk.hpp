#ifndef CHUNK_H
#define CHUNK_H

#include <array>
#include <vector>
#include <cstddef>
#include <functional>

enum VoxelState
{
    UNKNOWN,
    EMPTY,
    OCCUPIED
};

class Chunk{
    public:
        Chunk();
        Chunk(int res);
        VoxelState get_voxel_state(std::array<int,3> ind) const;
        void set_voxel_state(std::array<int,3> ind, VoxelState state);
        
    private:
        int flatten(const std::array<int,3>& indices) const;
        std::array<int,3> unflatten(int i) const;

        const int _res;
        std::vector<VoxelState> _voxels;
};

#endif