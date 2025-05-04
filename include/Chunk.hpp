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
        int flatten(const std::array<int,3>& indices) const;
        std::array<int,3> unflatten(int i) const;
        VoxelState getVoxelState(int i) const;
        void setVoxelState(int i, VoxelState state) const;

    private:
        const int _res;
        std::vector<VoxelState> _voxels;
};

#endif