#include <gtest/gtest.h>
#include "CostMap.hpp"

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
    CostMap costMap(0.25,{0,0,0});
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {5, 2, 2};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getNumChunks(), 2);
    EXPECT_EQ(costMap.getVoxelState({5,3,3}), VoxelState::EMPTY);
    EXPECT_EQ(costMap.getVoxelState({5,5,5}), VoxelState::UNKNOWN);
    EXPECT_EQ(costMap.getVoxelState({1,1,1}), VoxelState::OCCUPIED);
}

TEST(CostMapTests, NegativeWorldCoordinates)
{
    CostMap costMap;
    std::array<float,3> xyz_min = {-2, 1, 0};
    std::array<float,3> xyz_max {1, 3, 2};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getNumChunks(), 2);
    // EXPECT_EQ(costMap.getVoxelState({5,3,3}), VoxelState::EMPTY);
    // EXPECT_EQ(costMap.getVoxelState({5,5,5}), VoxelState::UNKNOWN);
    // EXPECT_EQ(costMap.getVoxelState({1,1,1}), VoxelState::OCCUPIED);

    CostMap costMap2;
    costMap2.setVoxelState({-20,-20,-20}, VoxelState::OCCUPIED);
    EXPECT_EQ(costMap2.getNumChunks(), 2);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}