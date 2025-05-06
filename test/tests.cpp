#include <gtest/gtest.h>
#include "CostMap.hpp"

TEST(CostMapTests, CostMapStuff)
{
    std::array<float,3> mapOffset({0,0,0});
    CostMap costMap(1, mapOffset);
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {4, 4, 4};
    costMap.addObstacle(xyz_min, xyz_max);
    // std::cerr << "this should print to terminal!\n";
    EXPECT_EQ(costMap.getVoxelState({2.5,2.5,3}), VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getVoxelState({1,1,1}), VoxelState::EMPTY);
    EXPECT_EQ(costMap.getVoxelState({2,2,2}), VoxelState::OCCUPIED);
    EXPECT_EQ(costMap.getNumChunks(), 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}