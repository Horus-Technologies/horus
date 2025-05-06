#include "CostMap.hpp"

using ChunkKey = std::array<int, 3>;

CostMap::CostMap() : _scale(1.0), _mapOffset({0,0,0}){
    Chunk chunk(_res);
    std::array<int, 3> intKey = {static_cast<int>(_mapOffset[0]),
        static_cast<int>(_mapOffset[1]),
        static_cast<int>(_mapOffset[2])};
    _map.emplace(intKey,chunk);
}

CostMap::CostMap(float scale, std::array<float, 3> mapOffset) : _scale(scale), _mapOffset(mapOffset){
    Chunk chunk(_res);
    std::array<int, 3> intKey = {static_cast<int>(_mapOffset[0]),
        static_cast<int>(_mapOffset[1]),
        static_cast<int>(_mapOffset[2])};
    _map.emplace(intKey,chunk);
}

// Convert from world coordinates to global voxel indices
std::array<int,3> CostMap::worldToGlobal(const std::array<float,3>& position) const
{
    return {
        std::round((position[0]-_mapOffset[0])/_scale - 0.5),
        std::round((position[1]-_mapOffset[1])/_scale - 0.5),
        std::round((position[2]-_mapOffset[2])/_scale - 0.5)
    };
}

// Convert from global voxel indices to world coordinates
std::array<float,3> CostMap::globalToWorld(const std::array<int,3>& global_indices) const
{
    // convert to base frame position before returning
    return {
        (global_indices[0]+0.5)*_scale + _mapOffset[0],
        (global_indices[1]+0.5)*_scale + _mapOffset[1],
        (global_indices[2]+0.5)*_scale + _mapOffset[2]
    };
}

// Convert global voxel indices to chunk indices and local indices within chunk, respectively
std::pair<std::array<int,3>,std::array<int,3>> CostMap::globalToLocal(const std::array<int,3>& global_indices) const
{
    std::array<int,3> chunk_indices = {
        global_indices[0] / _res,
        global_indices[1] / _res,
        global_indices[2] / _res
    };

    std::array<int,3> local_indices = {
        global_indices[0] % _res,
        global_indices[1] % _res,
        global_indices[2] % _res
    };

    return {chunk_indices, local_indices};
}


std::array<int,3> CostMap::localToGlobal(const std::array<int,3>& chunk_indices,const std::array<int,3>& local_indices) const
{
    return {
        chunk_indices[0]*_res + local_indices[0],
        chunk_indices[1]*_res + local_indices[1],
        chunk_indices[2]*_res + local_indices[2]
    };
}


// Returns VoxelState at the world coordinates provided
VoxelState CostMap::getVoxelState(const std::array<float,3>& position) const
{
    // Obtain global voxel indices
    std::array<int,3> global = worldToGlobal(position);

    // Obtain chunk indices
    auto p = globalToLocal(global);
    std::array<int,3> chunk = p.first;
    std::array<int,3> local = p.second;
    // std::cout << "DESIRED CHUNK INDICES ARE " << chunk[0] << " " << chunk[1] << " " << chunk[2] << std::endl;
    // std::cout << "ACTUAL CHUNK INDICES ARE " << _map.begin()->first[0] << " " << _map.begin()->first[1] << " " << _map.begin()->first[2] << std::endl;
    // Check if chunk exists
    if (_map.find(chunk) != _map.end()){
        // Obtain local indices within chunk

        const Chunk* chunk_ptr = &_map.at(chunk);
        return chunk_ptr->getVoxelState(local);
    }
    else{ // chunk doesn't exist
        return VoxelState::UNKNOWN;
    }
}

// Sets VoxelState at the world coordinates provided
void CostMap::setVoxelState(std::array<float,3> position, VoxelState state)
{
    // Obtain global voxel indices
    std::array<int,3> global = worldToGlobal(position);

    // Obtain chunk indices
    auto p = globalToLocal(global);
    std::array<int,3> chunk = p.first;
    std::array<int,3> local = p.second;
    if (_map.find(chunk) == _map.end()){
        // chunk doesn't exist, so make new chunk
        Chunk new_chunk(_res);
        _map.emplace(chunk, new_chunk);
    }

    Chunk* chunk_ptr = &_map.at(chunk);
    chunk_ptr->setVoxelState(local, state);
}


// NEEDS FIXING BASED ON POSITION FRAMES
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

    std::array<int,3> xyz_min_aligned_ind = worldToGlobal(xyz_min_aligned);
    std::array<int,3> xyz_max_aligned_ind = worldToGlobal(xyz_max_aligned);

    for (int i = xyz_min_aligned_ind[0]; i <= xyz_max_aligned_ind[0]; i++){
        for (int j = xyz_min_aligned_ind[1]; j <= xyz_max_aligned_ind[1]; j++){
            for (int k = xyz_min_aligned_ind[2]; k <= xyz_max_aligned_ind[2]; k++){
                setVoxelState(globalToWorld({i,j,k}), VoxelState::OCCUPIED);
            }
        }
    }
}

const std::optional<std::vector<std::array<float,3>>> CostMap::emptyNeighbors(const std::array<float,3>& position) const
{
    std::vector<std::array<float,3>> emp_neighbors; // world coordinates of empty neighbors

    std::array<int,3> global = worldToGlobal(position);
    auto p = globalToLocal(global);
    std::array<int,3> chunk = p.first;
    std::array<int,3> local = p.second;

    // check if chunk doesn't exist
    if (_map.find(chunk) == _map.end()){
        return std::nullopt; // represents "no value"
    }
    // At this point, it is known that chunk exists in map
    
    const std::array<std::array<int, 3>, 6> neighborOffsets = {{
        { 1,  0,  0},  // +x
        {-1,  0,  0},  // -x
        { 0,  1,  0},  // +y
        { 0, -1,  0},  // -y
        { 0,  0,  1},  // +z
        { 0,  0, -1},  // -z
    }};

    // returns how chunk needs to be adjusted for neighbor voxel
    auto chunkDelta = [&](int x, int y, int z) {
        std::array<int,3> delta = {0,0,0};
        if (x < 0){ delta[0] = -1;}
        else if (x > _res-1){ delta[0] = 1;}

        if (y < 0){ delta[1] = -1;}
        else if (y > _res-1){ delta[1] = 1;}

        if (z < 0){ delta[2] = -1;}
        else if (z > _res-1){ delta[2] = 1;}

        return delta;
    };
    

    // Loop through neighbor offsets
    for (const auto& offset : neighborOffsets) {
        // Obtain neighbor local voxel indices
        int nx = local[0] + offset[0];
        int ny = local[1] + offset[1];
        int nz = local[2] + offset[2];

        std::array<int,3> chunkDelta = {nx,ny,nz};
        if (chunkDelta == std::array<int, 3>{0,0,0}){ // neighbor within same chunk as original voxel
            if(_map.at(chunk).getVoxelState({nx,ny,nz}) == VoxelState::EMPTY){
                std::array<int,3> n_global = localToGlobal(chunk,{nx,ny,nz});
                std::array<float,3> n_wold = globalToWorld(n_global);
                emp_neighbors.push_back(n_wold);
            }
        }
        else{
            // adjust chunk
            chunk[0] += chunkDelta[0];
            chunk[1] += chunkDelta[1];
            chunk[2] += chunkDelta[2];

            // check if chunk exists
            if (_map.find(chunk) != _map.end()){
                if(_map.at(chunk).getVoxelState({nx,ny,nz}) == VoxelState::EMPTY){
                    std::array<int,3> n_global = localToGlobal(chunk,{nx,ny,nz});
                    std::array<float,3> n_wold = globalToWorld(n_global);
                    emp_neighbors.push_back(n_wold);
                }
            }
        }
    }
    
    return emp_neighbors;
}

// Amanatides & Woo Algorithm to traverse line of sight segment
bool CostMap::checkCollision(const std::array<float,3>& point1, const std::array<float,3>& point2) const
{
    // Initialization
    Eigen::Vector3f A(point1[0],point1[1],point1[2]);
    Eigen::Vector3f B(point2[0],point2[1],point2[2]);
    Eigen::Vector3f v = (B-A) / (B-A).norm(); // unit vector
    std::array<int,3> voxelA = worldToGlobal(point1);
    std::array<int,3> voxelB = worldToGlobal(point2);
    int X = voxelA[0];
    int Y = voxelA[1];
    int Z = voxelA[2];
    int stepX = (0 < v(0)) - (0 > v(0)); // 1 for positive x direction, -1 for negative, and 0 for neutral
    int stepY = (0 < v(1)) - (0 > v(1));
    int stepZ = (0 < v(2)) - (0 > v(2));
    float tMaxX;
    float tMaxY;
    float tMaxZ;
    std::array<float,3> centerStart = globalToWorld(voxelA);
    if (v(0) == 0)
    {
        tMaxX = std::numeric_limits<float>::max();
    }
    else{
        float xDist = centerStart[0] + _scale/2 - A[0];
        tMaxX = (v * xDist/v(0)).norm(); // works only if ray starts from center of voxel
    }
    if (v(1) == 0)
    {
        tMaxY = std::numeric_limits<float>::max();
    }
    else{
        float yDist = centerStart[1] + _scale/2 - A[1];
        tMaxY = (v * yDist/v(1)).norm(); // works only if ray starts from center of voxel
    }
    if (v(2) == 0)
    {
        tMaxZ = std::numeric_limits<float>::max();
    }
    else{
        float zDist = centerStart[2] + _scale/2 - A[2];
        tMaxZ = (v * zDist/v(2)).norm(); // works only if ray starts from center of voxel
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

        
        if (count == 10000)
        {
            throw std::runtime_error("Voxel traversal during collision check taking too long (count = 10000)");
        }
        // std::cout << " X Y Z: " << X << " " << Y << " " << Z << std::endl;
        std::array<float,3> pos = globalToWorld({X,Y,Z});
        VoxelState voxelState = getVoxelState(pos); // INEFFICIENT
        if (voxelState == VoxelState::OCCUPIED){
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

void CostMap::forEachVoxel(const std::function<void(float x, float y, float z)>& func)
{
    for (const auto& pair : _map){
        const ChunkKey chunk_indices = pair.first;
        const Chunk* chunk = &(pair.second); 

        for (int i = 0; i < _res; ++i)
        {
            for (int j = 0; j < _res; ++j)
            {
                for (int k = 0; k < _res; ++k)
                {
                    std::array<int,3> global_indices = localToGlobal(chunk_indices,{i,j,k});
                    auto [x, y, z] = globalToWorld(global_indices);
                    func(x,y,z);
                }
            }
        }
    }
}