#include "CostMap.hpp"

using ChunkKey = std::array<int, 3>;

CostMap::CostMap() : _scale(1.0){
    Chunk chunk(_res);
    std::array<int, 3> originKey = {0,0,0};
    _map.emplace(originKey,chunk);
}

CostMap::CostMap(float scale) : _scale(scale){
    Chunk chunk(_res);
    std::array<int, 3> originKey = {0,0,0};
    _map.emplace(originKey,chunk);
}

// Convert from world coordinates to global voxel indices
std::array<int,3> CostMap::worldToGlobal(const std::array<float,3>& position) const
{
    return {
        std::floor((position[0])/_scale),
        std::floor((position[1])/_scale),
        std::floor((position[2])/_scale)
    };
}

// Convert from global voxel indices to world coordinates
std::array<float,3> CostMap::globalToWorld(const std::array<int,3>& global_indices) const
{
    // convert to base frame position before returning
    return {
        (global_indices[0]+0.5)*_scale,
        (global_indices[1]+0.5)*_scale,
        (global_indices[2]+0.5)*_scale
    };
}

// Convert global voxel indices to chunk indices and local indices within chunk, respectively
std::pair<std::array<int,3>,std::array<int,3>> CostMap::globalToLocal(const std::array<int,3>& global_indices) const
{
    std::array<int,3> chunk_indices = {
        std::floor(static_cast<double>(global_indices[0]) / _res),
        std::floor(static_cast<double>(global_indices[1]) / _res),
        std::floor(static_cast<double>(global_indices[2]) / _res)
    };

    std::array<int,3> local_indices = {
        global_indices[0] - chunk_indices[0]*_res,
        global_indices[1] - chunk_indices[1]*_res,
        global_indices[2] - chunk_indices[2]*_res
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
    
    // Check if chunk exists
    if (_map.find(chunk) != _map.end()){
        // Obtain local indices within chunk

        const Chunk* chunk_ptr = &_map.at(chunk);
        return chunk_ptr->getVoxelState(local);
    }
    else{ // chunk doesn't exist
        return VoxelState::EMPTY;
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

    for (int i = xyz_min_aligned_ind[0]; i < xyz_max_aligned_ind[0]; i++){
        for (int j = xyz_min_aligned_ind[1]; j < xyz_max_aligned_ind[1]; j++){
            for (int k = xyz_min_aligned_ind[2]; k < xyz_max_aligned_ind[2]; k++){
                setVoxelState(globalToWorld({i,j,k}), VoxelState::OCCUPIED);
            }
        }
    }
}

// Returns vector of global indices for empty neighbors.
const std::optional<std::vector<std::array<int,3>>> CostMap::emptyNeighbors(const std::array<int,3>& global) const
{
    std::vector<std::array<int,3>> emp_neighbors; // world coordinates of empty neighbors
    
    const std::array<std::array<int, 3>, 6> neighborOffsets = {{
        { 1,  0,  0},  // +x
        {-1,  0,  0},  // -x
        { 0,  1,  0},  // +y
        { 0, -1,  0},  // -y
        { 0,  0,  1},  // +z
        { 0,  0, -1},  // -z
    }};    

    // Loop through neighbor offsets
    for (const auto& offset : neighborOffsets) {
        // Obtain neighbor global voxel indices
        std::array<int,3> global_n;
        global_n[0] = global[0] + offset[0];
        global_n[1] = global[1] + offset[1];
        global_n[2] = global[2] + offset[2];

        auto p = globalToLocal(global_n);
        std::array<int,3> chunk_n = p.first;
        std::array<int,3> local_n = p.second;

        if (_map.find(chunk_n) != _map.end()){
            // neighbor is in a chunk that exists, so check to make sure voxel EMPTY
            if(_map.at(chunk_n).getVoxelState(local_n) == VoxelState::EMPTY){
                emp_neighbors.push_back(global_n);
            }
        }   
        else{
            // simply add neighbor as empty, since chunk doesn't exist
            emp_neighbors.push_back(global_n);
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
    std::cout << "Point A: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
    std::cout << "Point B: " << point2[0] << " " << point2[1] << " " << point2[2] << std::endl;
    std::cout << "v: " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    std::cout << "Voxel A: " << voxelA[0] << " " << voxelA[1] << " " << voxelA[2] << std::endl;
    std::cout << "Voxel B: " << voxelB[0] << " " << voxelB[1] << " " << voxelB[2] << std::endl;

    int stepX = (v(0) > 0) ? 1 : (v(0) < 0) ? -1 : 0; // 1 for positive x direction, -1 for negative, and 0 for neutral
    int stepY = (v(1) > 0) ? 1 : (v(1) < 0) ? -1 : 0;
    int stepZ = (v(2) > 0) ? 1 : (v(2) < 0) ? -1 : 0;
    float tMaxX=0;
    float tMaxY=0;
    float tMaxZ=0;
    std::array<float,3> centerStart = globalToWorld(voxelA);
    std::cout << "centerStart: " << centerStart[0] << " " << centerStart[1] << " " << centerStart[2] << std::endl;
    if (v(0) == 0)
    {
        tMaxX = std::numeric_limits<float>::max();
    }
    else{
        float xDist = centerStart[0] + (stepX*_scale/2) - A[0]; // distance to next x plane
        // tMaxX = (v * xDist/v(0)).norm();
        tMaxX = std::abs(xDist/v(0));
    }
    if (v(1) == 0)
    {
        tMaxY = std::numeric_limits<float>::max();
    }
    else{
        float yDist = centerStart[1] + (stepY*_scale/2) - A[1];
        // tMaxY = (v * yDist/v(1)).norm();
        tMaxY = std::abs(yDist/v(1));
    }
    if (v(2) == 0)
    {
        tMaxZ = std::numeric_limits<float>::max();
    }
    else{
        float zDist = centerStart[2] + (stepZ*_scale/2) - A[2];
        // tMaxZ = (v * zDist/v(2)).norm();
        tMaxZ = std::abs(zDist/v(2));
    }

     auto nearlyEqual = [](float a, float b, float eps) {
            return std::abs(a - b) < eps;
        };

    float tDeltaX = std::abs(_scale/v(0));
    float tDeltaY = std::abs(_scale/v(1));
    float tDeltaZ = std::abs(_scale/v(2));

    bool reachedVoxelB = false;

    int count = 0;
    // Incremental Traversal
    while(!reachedVoxelB)
    {
        std::cout << "tMaxes: " << tMaxX << " " << tMaxY << " " << tMaxZ << std::endl;
        // Check if all equal to each other, this handles edge case of point B in corner of voxel
        
        constexpr float epsilon = 1e-5f;
        if (nearlyEqual(tMaxX, tMaxY, epsilon) && nearlyEqual(tMaxX, tMaxZ, epsilon)) {
            reachedVoxelB = true;
        }


        // check if voxelB has been reached
        if(X==voxelB[0] && Y==voxelB[1] && Z==voxelB[2]){
            reachedVoxelB = true;
        }

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

        std::cout << " X Y Z: " << X << " " << Y << " " << Z << std::endl;
        
        if (count == 1000)
        {
            throw std::runtime_error("Voxel traversal during collision check taking too long (count = 1000)");
        }
        
        std::array<float,3> pos = globalToWorld({X,Y,Z});
        VoxelState voxelState = getVoxelState(pos); // INEFFICIENT
        if (voxelState == VoxelState::OCCUPIED){
            // std::cout << "COLLISION FOUND" << std::endl;
            return true;
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

std::vector<std::array<int,3>> CostMap::getChunkIndices() const
{
    std::vector<std::array<int,3>> chunks;
    for (const auto& pair : _map){
        const ChunkKey chunk_indices = pair.first;
        chunks.push_back(chunk_indices);
    }

    return chunks;
}

// Pair of min xyz and max xyz that show chunk-wise map limits in rectangular prism fashion
std::pair<std::array<float,3>, std::array<float,3>> CostMap::mapLimits(const std::array<float,3>& start, const std::array<float,3>& goal) const
{
    int maxChunk_x = 0;
    int maxChunk_y = 0;
    int maxChunk_z = 0;

    int minChunk_x = 0;
    int minChunk_y = 0;
    int minChunk_z = 0;
    
    for (const auto& pair : _map){
        const ChunkKey chunk_indices = pair.first;
        
        if(chunk_indices[0] > maxChunk_x){maxChunk_x = chunk_indices[0];}
        if(chunk_indices[1] > maxChunk_y){maxChunk_y = chunk_indices[1];}
        if(chunk_indices[2] > maxChunk_z){maxChunk_z = chunk_indices[2];}

        if(chunk_indices[0] < minChunk_x){minChunk_x = chunk_indices[0];}
        if(chunk_indices[1] < minChunk_y){minChunk_y = chunk_indices[1];}
        if(chunk_indices[2] < minChunk_z){minChunk_z = chunk_indices[2];}
    }

    std::array<float,3> minXYZ = {
        _res*_scale*minChunk_x,
        _res*_scale*minChunk_y,
        _res*_scale*minChunk_z
    };
    std::array<float,3> maxXYZ = {
        _res*_scale*(maxChunk_x+1),
        _res*_scale*(maxChunk_y+1),
        _res*_scale*(maxChunk_z+1)
    };

    minXYZ[0] = std::min({minXYZ[0],start[0],goal[0]});
    minXYZ[1] = std::min({minXYZ[1],start[1],goal[1]});
    minXYZ[2] = std::min({minXYZ[2],start[2],goal[2]});

    maxXYZ[0] = std::max({maxXYZ[0],start[0],goal[0]});
    maxXYZ[1] = std::max({maxXYZ[1],start[1],goal[1]});
    maxXYZ[2] = std::max({maxXYZ[2],start[2],goal[2]});

    return {minXYZ, maxXYZ};
}
