#include "VoxelGrid.hpp"

using ChunkKey = std::array<int, 3>;

VoxelGrid::VoxelGrid() : _scale(1.0){
    Chunk chunk(_res);
    std::array<int, 3> origin_key = {0,0,0};
    _map.emplace(origin_key,chunk);
}

VoxelGrid::VoxelGrid(float scale) : _scale(scale){
    Chunk chunk(_res);
    std::array<int, 3> origin_key = {0,0,0};
    _map.emplace(origin_key,chunk);
}

// Convert from world coordinates to global voxel indices
std::array<int,3> VoxelGrid::world_to_global(const std::array<float,3>& position) const
{
    return {
        std::floor((position[0])/_scale),
        std::floor((position[1])/_scale),
        std::floor((position[2])/_scale)
    };
}

// Convert from global voxel indices to world coordinates
std::array<float,3> VoxelGrid::global_to_world(const std::array<int,3>& global_indices) const
{
    // convert to base frame position before returning
    return {
        (global_indices[0]+0.5)*_scale,
        (global_indices[1]+0.5)*_scale,
        (global_indices[2]+0.5)*_scale
    };
}

// Convert global voxel indices to chunk indices and local indices within chunk, respectively
std::pair<std::array<int,3>,std::array<int,3>> VoxelGrid::global_to_local(const std::array<int,3>& global_indices) const
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

std::array<int,3> VoxelGrid::local_to_global(const std::array<int,3>& chunk_indices,const std::array<int,3>& local_indices) const
{
    return {
        chunk_indices[0]*_res + local_indices[0],
        chunk_indices[1]*_res + local_indices[1],
        chunk_indices[2]*_res + local_indices[2]
    };
}

// Returns VoxelState at the world coordinates provided
VoxelState VoxelGrid::get_voxel_state(const std::array<float,3>& position) const
{
    // Obtain global voxel indices
    std::array<int,3> global = world_to_global(position);

    // Obtain chunk indices
    auto p = global_to_local(global);
    std::array<int,3> chunk = p.first;
    std::array<int,3> local = p.second;
    
    // Check if chunk exists
    if (_map.find(chunk) != _map.end()){
        // Obtain local indices within chunk

        const Chunk* chunk_ptr = &_map.at(chunk);
        return chunk_ptr->get_voxel_state(local);
    }
    else{ // chunk doesn't exist
        return VoxelState::EMPTY;
    }
}

// Sets VoxelState at the world coordinates provided
void VoxelGrid::set_voxel_state(std::array<float,3> position, VoxelState state)
{
    // Obtain global voxel indices
    std::array<int,3> global = world_to_global(position);

    // Obtain chunk indices
    auto p = global_to_local(global);
    std::array<int,3> chunk = p.first;
    std::array<int,3> local = p.second;
    if (_map.find(chunk) == _map.end()){
        // chunk doesn't exist, so make new chunk
        Chunk new_chunk(_res);
        _map.emplace(chunk, new_chunk);
    }

    Chunk* chunk_ptr = &_map.at(chunk);
    chunk_ptr->set_voxel_state(local, state);
}

// Adds rectangular prism obstacle defined by xyz_min and xyz_max
void VoxelGrid::add_obstacle(std::array<float,3> xyz_min, std::array<float,3> xyz_max)
{   
    // compute xyz limits for obstacle that align with grid
    std::array<float,3> xyz_min_aligned;
    std::array<float,3> xyz_max_aligned;
    for (int i=0; i < 3; i++)
    {
        xyz_min_aligned[i] = std::floor(xyz_min[i]/_scale) * _scale;
        xyz_max_aligned[i] = std::ceil(xyz_max[i]/_scale) * _scale;
    }

    std::array<int,3> xyz_min_aligned_ind = world_to_global(xyz_min_aligned);
    std::array<int,3> xyz_max_aligned_ind = world_to_global(xyz_max_aligned);

    for (int i = xyz_min_aligned_ind[0]; i < xyz_max_aligned_ind[0]; i++){
        for (int j = xyz_min_aligned_ind[1]; j < xyz_max_aligned_ind[1]; j++){
            for (int k = xyz_min_aligned_ind[2]; k < xyz_max_aligned_ind[2]; k++){
                set_voxel_state(global_to_world({i,j,k}), VoxelState::OCCUPIED);
            }
        }
    }
}

// Returns vector of global indices for empty neighbors.
const std::optional<std::vector<std::array<int,3>>> VoxelGrid::empty_neighbors(const std::array<int,3>& global) const
{
    std::vector<std::array<int,3>> empty_neighbors; // world coordinates of empty neighbors
    
    const std::array<std::array<int, 3>, 6> neighbor_offsets = {{
        { 1,  0,  0},  // +x
        {-1,  0,  0},  // -x
        { 0,  1,  0},  // +y
        { 0, -1,  0},  // -y
        { 0,  0,  1},  // +z
        { 0,  0, -1},  // -z
    }};    

    for (const auto& offset : neighbor_offsets) {
        // Obtain neighbor global voxel indices
        std::array<int,3> global_neighbor;
        global_neighbor[0] = global[0] + offset[0];
        global_neighbor[1] = global[1] + offset[1];
        global_neighbor[2] = global[2] + offset[2];

        auto p = global_to_local(global_neighbor);
        std::array<int,3> chunk_neighbor = p.first;
        std::array<int,3> local_neighbor = p.second;

        if (_map.find(chunk_neighbor) != _map.end()){
            // neighbor is in a chunk that exists, so check to make sure voxel EMPTY
            if(_map.at(chunk_neighbor).get_voxel_state(local_neighbor) == VoxelState::EMPTY){
                empty_neighbors.push_back(global_neighbor);
            }
        }   
        else{
            // simply add neighbor as empty, since chunk doesn't exist
            empty_neighbors.push_back(global_neighbor);
        }
    }

    return empty_neighbors;
}

// Amanatides & Woo Algorithm to traverse line of sight segment
bool VoxelGrid::check_collision(const std::array<float,3>& point1, const std::array<float,3>& point2) const
{
    // Initialization
    Eigen::Vector3f a(point1[0],point1[1],point1[2]);
    Eigen::Vector3f b(point2[0],point2[1],point2[2]);
    Eigen::Vector3f v = (b-a) / (b-a).norm(); // unit vector
    std::array<int,3> voxel_a = world_to_global(point1);
    std::array<int,3> voxel_b = world_to_global(point2);
    int x = voxel_a[0];
    int y = voxel_a[1];
    int z = voxel_a[2];
    // std::cout << "Point A: " << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
    // std::cout << "Point B: " << point2[0] << " " << point2[1] << " " << point2[2] << std::endl;
    // std::cout << "v: " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    // std::cout << "Voxel A: " << voxel_a[0] << " " << voxel_a[1] << " " << voxel_a[2] << std::endl;
    // std::cout << "Voxel B: " << voxel_b[0] << " " << voxel_b[1] << " " << voxel_b[2] << std::endl;

    int step_x = (v(0) > 0) ? 1 : (v(0) < 0) ? -1 : 0; // 1 for positive x direction, -1 for negative, and 0 for neutral
    int step_y = (v(1) > 0) ? 1 : (v(1) < 0) ? -1 : 0;
    int step_z = (v(2) > 0) ? 1 : (v(2) < 0) ? -1 : 0;
    float t_max_x=0;
    float t_max_y=0;
    float t_max_z=0;
    std::array<float,3> center_start = global_to_world(voxel_a);
    if (v(0) == 0)
    {
        t_max_x = std::numeric_limits<float>::max();
    }
    else{
        float dist_x = center_start[0] + (step_x*_scale/2) - a[0]; // distance to next x plane
        t_max_x = std::abs(dist_x/v(0));
    }
    if (v(1) == 0)
    {
        t_max_y = std::numeric_limits<float>::max();
    }
    else{
        float dist_y = center_start[1] + (step_y*_scale/2) - a[1];
        t_max_y = std::abs(dist_y/v(1));
    }
    if (v(2) == 0)
    {
        t_max_z = std::numeric_limits<float>::max();
    }
    else{
        float dist_z = center_start[2] + (step_z*_scale/2) - a[2];
        t_max_z = std::abs(dist_z/v(2));
    }

    auto nearly_equal = [](float a, float b, float eps) {
        return std::abs(a - b) < eps;
    };

    float t_delta_x = std::abs(_scale/v(0));
    float t_delta_y = std::abs(_scale/v(1));
    float t_delta_z = std::abs(_scale/v(2));

    bool reached_voxel_b = false;

    int count = 0;
    // Incremental Traversal
    while(!reached_voxel_b)
    {
        // Check if all equal to each other, this handles edge case of point B in corner of voxel
        constexpr float epsilon = 1e-5f;
        if (nearly_equal(t_max_x, t_max_y, epsilon) && nearly_equal(t_max_x, t_max_z, epsilon)) {
            reached_voxel_b = true;
        }

        // check if voxel_b has been reached
        if(x==voxel_b[0] && y==voxel_b[1] && z==voxel_b[2]){
            reached_voxel_b = true;
        }

        if(t_max_x < t_max_y && t_max_x < t_max_z) //t_max_x is smallest
        {   
            t_max_x = t_max_x + t_delta_x;
            x = x + step_x;
        }
        else if (t_max_y < t_max_z) //t_max_y is smallest
        {
            t_max_y = t_max_y + t_delta_y;
            y = y + step_y;
        }
        else //t_max_z is smallest
        {
            t_max_z = t_max_z + t_delta_z;
            z = z + step_z;
        }
        
        if (count == 1000)
        {
            throw std::runtime_error("Voxel traversal during collision check taking too long (count = 1000)");
        }
        
        std::array<float,3> pos = global_to_world({x,y,z});
        VoxelState voxel_state = get_voxel_state(pos);
        if (voxel_state == VoxelState::OCCUPIED){
            return true;
        }
        
        count++;
    }
    return false;
}

void VoxelGrid::for_each_voxel(const std::function<void(float x, float y, float z)>& func)
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
                    std::array<int,3> global_indices = local_to_global(chunk_indices,{i,j,k});
                    auto [x, y, z] = global_to_world(global_indices);
                    func(x,y,z);
                }
            }
        }
    }
}

std::vector<std::array<int,3>> VoxelGrid::get_chunk_indices() const
{
    std::vector<std::array<int,3>> chunks;
    for (const auto& pair : _map){
        const ChunkKey chunk_indices = pair.first;
        chunks.push_back(chunk_indices);
    }

    return chunks;
}

// Pair of min xyz and max xyz that show chunk-wise map limits in rectangular prism fashion
std::pair<std::array<float,3>, std::array<float,3>> VoxelGrid::map_limits(const std::array<float,3>& start, const std::array<float,3>& goal) const
{
    int max_chunk_x = 0;
    int max_chunk_y = 0;
    int max_chunk_z = 0;

    int min_chunk_x = 0;
    int min_chunk_y = 0;
    int min_chunk_z = 0;
    
    for (const auto& pair : _map){
        const ChunkKey chunk_indices = pair.first;
        
        if(chunk_indices[0] > max_chunk_x){max_chunk_x = chunk_indices[0];}
        if(chunk_indices[1] > max_chunk_y){max_chunk_y = chunk_indices[1];}
        if(chunk_indices[2] > max_chunk_z){max_chunk_z = chunk_indices[2];}

        if(chunk_indices[0] < min_chunk_x){min_chunk_x = chunk_indices[0];}
        if(chunk_indices[1] < min_chunk_y){min_chunk_y = chunk_indices[1];}
        if(chunk_indices[2] < min_chunk_z){min_chunk_z = chunk_indices[2];}
    }

    std::array<float,3> min_xyz = {
        _res*_scale*min_chunk_x,
        _res*_scale*min_chunk_y,
        _res*_scale*min_chunk_z
    };
    std::array<float,3> max_xyz = {
        _res*_scale*(max_chunk_x+1),
        _res*_scale*(max_chunk_y+1),
        _res*_scale*(max_chunk_z+1)
    };

    min_xyz[0] = std::min({min_xyz[0],start[0],goal[0]});
    min_xyz[1] = std::min({min_xyz[1],start[1],goal[1]});
    min_xyz[2] = std::min({min_xyz[2],start[2],goal[2]});

    max_xyz[0] = std::max({max_xyz[0],start[0],goal[0]});
    max_xyz[1] = std::max({max_xyz[1],start[1],goal[1]});
    max_xyz[2] = std::max({max_xyz[2],start[2],goal[2]});

    return {min_xyz, max_xyz};
}
