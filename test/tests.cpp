#include <gtest/gtest.h>
#include "CostMap.hpp"

TEST(CostMapTests, FrameConversions)
{
    CostMap costMap; // defaults to 1 scale and 16 voxels resolution;
    // Origin conversion
    std::array<int,3> global = costMap.worldToGlobal({0.0,0.0,0.0});
    std::array<int,3> expected_global = {0,0,0};
    EXPECT_EQ(global, expected_global);

    // Conversion world <--> global
    global = costMap.worldToGlobal({2.3,1.3,0.0});
    expected_global = {2,1,0};
    EXPECT_EQ(global, expected_global);
    std::array<float,3> world = costMap.globalToWorld(global);
    std::array<float,3> expected_world = {2.5,1.5,0.5};
    EXPECT_EQ(world, expected_world);

    // Conversion global -> local
    global = costMap.worldToGlobal({16.0,0.0,0.0});
    expected_global = {16,0,0};
    EXPECT_EQ(global, expected_global);
    auto [chunk, local] = costMap.globalToLocal(global);
    std::array<int,3> expected_chunk = {1,0,0};
    std::array<int,3> expected_local = {0,0,0};
    EXPECT_EQ(chunk, expected_chunk);
    EXPECT_EQ(local, expected_local);

    // Conversion local -> global
    global = costMap.localToGlobal(chunk, local);
    expected_global = {16,0,0};
    EXPECT_EQ(global, expected_global);

    // Negative world coordinate conversions
    global = costMap.worldToGlobal({-18.0,0.0,0.0});
    expected_global = {-18,0,0};
    EXPECT_EQ(global, expected_global);
    auto p = costMap.globalToLocal(global);
    chunk = p.first;
    local = p.second;
    expected_chunk = {-2,0,0};
    expected_local = {14,0,0};
    EXPECT_EQ(chunk, expected_chunk);
    EXPECT_EQ(local, expected_local);
}


TEST(CostMapTests, DefaultConstructor)
{
    CostMap costMap;
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {4, 4, 4};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getVoxelState({2.5,2.5,3}), VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getVoxelState({0.5,0.5,0.5}), VoxelState::EMPTY);
    EXPECT_EQ(costMap.getVoxelState({2,2,2}), VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getNumChunks(), 1);

    // Add voxel far away so new chunk is created
    costMap.setVoxelState({20,20,20}, VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getVoxelState({20,20,20}), VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getNumChunks(), 2);
}

TEST(CostMapTests, ScaleChange)
{
    CostMap costMap(0.25);
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {5, 2, 2};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getNumChunks(), 2);
    EXPECT_EQ(costMap.getVoxelState({5,3,3}), VoxelState::EMPTY);
    EXPECT_EQ(costMap.getVoxelState({5,5,5}), VoxelState::UNKNOWN);
    EXPECT_EQ(costMap.getVoxelState({1,1,1}), VoxelState::OCCUPIED);
}

TEST(CostMapTests, NegativeWorldCoordinatesSetVoxel)
{
    CostMap costMap;
    costMap.setVoxelState({-3,1,1}, VoxelState::OCCUPIED);
    std::vector<std::array<int,3>> chunks = costMap.getChunkIndices();
    std::sort(chunks.begin(), chunks.end());
    EXPECT_EQ(costMap.getNumChunks(), 2);

    std::array<int,3> expected1 = {0, 0, 0};
    EXPECT_EQ(chunks[1], expected1);

    std::array<int,3> expected2 = {-1, 0, 0};
    EXPECT_EQ(chunks[0], expected2);

}

TEST(CostMapTests, NegativeWorldCoordinatesAddObstacle)
{
    CostMap costMap;
    std::array<float,3> xyz_min = {-2, 1, 0};
    std::array<float,3> xyz_max {1, 3, 2};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getNumChunks(), 2);
    std::vector<std::array<int,3>> chunks = costMap.getChunkIndices();
    std::sort(chunks.begin(),chunks.end());
    std::array<int,3> expected1 = {0, 0, 0};
    EXPECT_EQ(chunks[1], expected1);
    std::array<int,3> expected2 = {-1, 0, 0};
    EXPECT_EQ(chunks[0], expected2);
    // EXPECT_EQ(costMap.getVoxelState({5,3,3}), VoxelState::EMPTY);
    // EXPECT_EQ(costMap.getVoxelState({5,5,5}), VoxelState::UNKNOWN);
    // EXPECT_EQ(costMap.getVoxelState({1,1,1}), VoxelState::OCCUPIED);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}