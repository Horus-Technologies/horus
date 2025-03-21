#include <gtest/gtest.h>
#include "CostMap.hpp"

// Test Voxel
TEST(VoxelTests, VoxelCostDefault)
{
    Voxel voxel;
    double cost = voxel.getCost();
    EXPECT_EQ(cost, 0);

    double getPosition[3];
    voxel.getPosition();
    for (int i = 0; i < 3; i++)
    {
        EXPECT_EQ(getPosition[i], 0);
    }
}

TEST(VoxelTests, VoxelCost)
{
    std::array<int, 3> index = {1, 2, 3};
    double scale = 1;
    double cost = 4;
    Voxel voxel(index, cost, scale);
    EXPECT_EQ(voxel.getCost(), 4);

    std::array<double, 3> pos = voxel.getPosition();
    std::array<double,3> expectedPosition = {1.5, 2.5, 3.5};
    for (int i = 0; i < 3; i++)
    {
        EXPECT_EQ(pos[i], expectedPosition[i]);
    }
}

TEST(CostMapTests, CostMapStuff)
{
    CostMap costMap;
    std::array<double,3> xyz_min = {2, 2, 2};
    std::array<double,3> xyz_max {3, 3, 4};
    costMap.addObstacle(xyz_min, xyz_max);
    const std::vector<std::vector<std::vector<Voxel>>>& voxels = costMap.getVoxels();
    EXPECT_EQ(voxels.size(), 10); // default dims is 10
    EXPECT_EQ(voxels[2][2][2].getCost(), 1);

    // costMap.findVoxelByPosition()
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}