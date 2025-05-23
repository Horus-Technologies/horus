#include <gtest/gtest.h>
#include "CostMap.hpp"
#include "Search.hpp"

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
    world = costMap.globalToWorld({-1,0,0});
    expected_world = {-0.5,0.5,0.5};
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
    EXPECT_EQ(costMap.getVoxelState({5,5,5}), VoxelState::EMPTY);
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

TEST(CostMapTests, EmptyNeighbors)
{
    CostMap costMap;
    // No Obstacle
    auto neighbors = costMap.emptyNeighbors({4,4,4});
    EXPECT_TRUE(neighbors.has_value());
    EXPECT_EQ(neighbors.value().size(), 6);
    auto outside_neighbors = costMap.emptyNeighbors({100,100,100});
    EXPECT_TRUE(outside_neighbors.has_value());
    neighbors = costMap.emptyNeighbors({0,0,0});
    EXPECT_EQ(neighbors.value().size(), 6);

    // With Obstacle
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {4, 4, 4};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_EQ(costMap.getVoxelState({2,0.5,2}), VoxelState::EMPTY);
    auto working_neighbors = costMap.emptyNeighbors({2,0,2});
    ASSERT_TRUE(working_neighbors.has_value());
    EXPECT_EQ(working_neighbors.value().size(), 5);
    std::sort(working_neighbors.value().begin(), working_neighbors.value().end());
    std::vector<std::array<int,3>> expected_neighbors;
    expected_neighbors.push_back({1,0,2});
    expected_neighbors.push_back({2,-1,2});
    expected_neighbors.push_back({2,0,1});
    expected_neighbors.push_back({2,0,3});
    expected_neighbors.push_back({3,0,2});
    for (int i = 0; i < working_neighbors.value().size(); i++){
        EXPECT_EQ(working_neighbors.value()[i], expected_neighbors[i]);
    }
}

TEST(CostMapTests, EmptyNeighborsChunkBoundary)
{
    CostMap costMap;
    auto neighbors = costMap.emptyNeighbors({0,0,0});
    EXPECT_TRUE(neighbors.has_value());
    EXPECT_EQ(neighbors.value().size(), 6);
    // std::sort(neighbors.value().begin(), neighbors.value().end());
    std::vector<std::array<int,3>> expected_neighbors;
    expected_neighbors.push_back({1,0,0});
    expected_neighbors.push_back({-1,0,0});
    expected_neighbors.push_back({0,1,0});
    expected_neighbors.push_back({0,-1,0});
    expected_neighbors.push_back({0,0,1});
    expected_neighbors.push_back({0,0,-1});
    for (int i = 0; i < neighbors.value().size(); i++){
        EXPECT_EQ(neighbors.value()[i], expected_neighbors[i]);
    }
}

TEST(CostMapTests, CheckCollision)
{
    CostMap costMap(0.25);
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {4, 4, 4};
    costMap.addObstacle(xyz_min, xyz_max);
    EXPECT_FALSE(costMap.checkCollision({5,5,5},{6,6,6}));
    // EXPECT_FALSE(costMap.checkCollision({5,5,5},{100,100,100}));
    // EXPECT_FALSE(costMap.checkCollision({50,50,50},{100,100,100}));
    // EXPECT_TRUE(costMap.checkCollision({5,5,5},{0,0,0}));
    // EXPECT_TRUE(costMap.checkCollision({5,5,5},{-50,-50,-50}));
    EXPECT_FALSE(costMap.checkCollision({-0.035556, 0.0518644, 0.194999},{-0.125, 0.125, 0.375}));
    EXPECT_FALSE(costMap.checkCollision({-0.035556, 0.0518644, 0.194999},{-0.125, 0.125, 0.625}));
    EXPECT_FALSE(costMap.checkCollision({0.894749, 1.00734, 5.98666},{1, 1, 6}));
}

// TEST(CostMapTests, MapLimits)
// {
//     CostMap costMap;
//     costMap.setVoxelState({-3,1,1}, VoxelState::OCCUPIED);
//     costMap.setVoxelState({20,1,1}, VoxelState::OCCUPIED);
//     auto p = costMap.mapLimits();
//     std::array<float,3> exp_min = {-16, 0, 0};
//     std::array<float,3> exp_max {32, 16, 16};
//     EXPECT_EQ(p.first,exp_min);
//     EXPECT_EQ(p.second,exp_max);
// }

TEST(SearchTests, CameFrom)
{
    std::array<int,3> dims = {16, 32, 48};
    Search::CameFrom came_from(dims);

    std::array<int,3> test_index = {15,10,8};
    std::array<int,3> default_index = {-1,-1,-1};
    EXPECT_EQ(came_from.at(test_index), default_index);

    std::array<int,3> expected_index = {14,12,11};
    came_from.set(test_index,expected_index);
    EXPECT_EQ(came_from.at(test_index), expected_index);
}

TEST(SearchTests, BreadthFirstSearch)
{
    CostMap costMap;
    std::array<float,3> start = {0,0,0};
    std::array<float,3> local_goal = {4,4,4};
    std::optional<std::vector<std::array<float,3>>> path = std::vector<std::array<float, 3>>{};
    int localRegionSize = 16;
    Search::runBreadthFirst(costMap, start, local_goal, path, localRegionSize);
    ASSERT_TRUE(path.has_value());
}

TEST(SearchTests, AStarInsideLocal)
{
    CostMap costMap;
    std::array<float,3> start = {0,0,0};
    std::array<float,3> local_goal = {4,4,4};
    std::optional<std::vector<std::array<float,3>>> path = std::vector<std::array<float, 3>>{};
    float localRegionSize = 16;
    Search::runAStar(costMap, start, local_goal, path, localRegionSize);
    ASSERT_TRUE(path.has_value());
    for (int i = 0; i < path.value().size(); i++){
        std::cout << path.value()[i][0] << ", " << path.value()[i][1] << ", " << path.value()[i][2] << std::endl;
    }
}

TEST(SearchTests, AStarOutsideLocal)
{
    CostMap costMap;
    std::array<float,3> start = {0,0,0};
    std::array<float,3> local_goal = {20,20,20};
    std::optional<std::vector<std::array<float,3>>> path = std::vector<std::array<float, 3>>{};
    float localRegionSize = 16;
    Search::runAStar(costMap, start, local_goal, path, localRegionSize);
    ASSERT_TRUE(path.has_value());
    for (int i = 0; i < path.value().size(); i++){
        std::cout << path.value()[i][0] << ", " << path.value()[i][1] << ", " << path.value()[i][2] << std::endl;
    }
}

// TEST(SearchTests, FindLocalGoal){
//     CostMap costMap;
//     std::array<float,3> start = {0,0,0};
//     std::array<float,3> goal = {20,20,20};
//     int localRegionSize = 16;
//     std::array<float,3> local_goal = Search::findLocalGoal(costMap, start, goal, localRegionSize);
//     std::array<float,3> expected_local_goal = {15.5,15.5,15.5};
//     EXPECT_EQ(local_goal, expected_local_goal);

//     goal = {20,0,0};
//     local_goal = Search::findLocalGoal(costMap, start, goal, localRegionSize);
//     expected_local_goal = {15.5,8,8};
//     EXPECT_EQ(local_goal, expected_local_goal);

//     goal = {20,0,0};
//     localRegionSize = 4;
//     local_goal = Search::findLocalGoal(costMap, start, goal, localRegionSize);
//     expected_local_goal = {3.5,2,2};
//     EXPECT_EQ(local_goal, expected_local_goal);
// }

// TEST(SearchTests, FindLocalGoalInside){
//     CostMap costMap;
//     std::array<float,3> start = {0,0,0};
//     std::array<float,3> goal = {5,5,5};
//     int localRegionSize = 16;
//     std::array<float,3> local_goal = Search::findLocalGoal(costMap, start, goal, localRegionSize);
//     std::array<float,3> expected_local_goal = {13,13,13};
//     EXPECT_EQ(local_goal, expected_local_goal);
// }

// TEST(SearchTests, FindLocalGoalObstacle){
//     CostMap costMap;
//     std::array<float,3> start = {0,0,0};
//     std::array<float,3> goal = {20,0,0};
//     int localRegionSize = 16;
//     std::array<float,3> xyz_min = {14, -2, -2};
//     std::array<float,3> xyz_max {18, 2, 2};
//     costMap.addObstacle(xyz_min, xyz_max);
//     std::array<float,3> local_goal = Search::findLocalGoal(costMap, start, goal, localRegionSize);
//     std::array<float,3> expected_local_goal = {15.5,8,8};
//     EXPECT_EQ(local_goal, expected_local_goal);
// }

TEST(SearchTests, RunSearchSimple)
{
    CostMap costMap;
    std::array<float,3> start = {0,0,0};
    std::array<float,3> goal = {4,0.3,0.3};
    auto path = Search::runSearch(costMap, start, goal);
    ASSERT_TRUE(path.has_value());
    std::vector<std::array<float,3>> expected_path;
    expected_path.push_back({0,0,0});
    expected_path.push_back({1.5,0.5,0.5});
    expected_path.push_back({2.5,0.5,0.5});
    expected_path.push_back({3.5,0.5,0.5});
    expected_path.push_back({4,0.3,0.3});
    for (int i = 0; i < path.value().size(); i++){
        EXPECT_EQ(path.value()[i], expected_path[i]);
    }
}

TEST(SearchTests, RunSearchAroundObstacle)
{
    CostMap costMap;
    std::array<float,3> xyz_min = {1, 1, 0};
    std::array<float,3> xyz_max {3, 3, 3};
    costMap.addObstacle(xyz_min, xyz_max);
    std::array<float,3> start = {2.5,0.5,1.5};
    std::array<float,3> goal = {3.5,3.5,1.5};
    auto path = Search::runSearch(costMap, start, goal);
    std::vector<std::array<float,3>> expected_path;
    expected_path.push_back({2.5,0.5,1.5});
    expected_path.push_back({3.5,0.5,1.5});
    expected_path.push_back({3.5,1.5,1.5});
    expected_path.push_back({3.5,2.5,1.5});
    expected_path.push_back({3.5,3.5,1.5});
    for (int i = 0; i < path.value().size(); i++){
        EXPECT_EQ(path.value()[i], expected_path[i]);
    }

    // Clean Path
    Search::cleanPath(costMap, path.value());
    EXPECT_EQ(path.value().size(), 3);
    expected_path.clear();
    expected_path.push_back({2.5,0.5,1.5});
    expected_path.push_back({3.5,0.5,1.5});
    expected_path.push_back({3.5,3.5,1.5});
    for (int i = 0; i < path.value().size(); i++){
        EXPECT_EQ(path.value()[i], expected_path[i]);
    }
}

TEST(SearchTests, CleanPath)
{
    CostMap costMap;
    std::array<float,3> start = {18,0,0};
    std::array<float,3> goal = {20.625,0.625,4.875};
    auto path = Search::runSearch(costMap, start, goal);
    Search::cleanPath(costMap, path.value());
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path.value().size(), 2);
    EXPECT_EQ(costMap.getNumChunks(), 1);
}

TEST(SearchTests, ImpossiblePathStartingInObstacle)
{
    CostMap costMap;
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {3, 3, 3};
    costMap.addObstacle(xyz_min, xyz_max);
    std::array<float,3> start = {2,2,2};
    std::array<float,3> goal = {3,3,3};
    auto path = Search::runSearch(costMap, start, goal);
    EXPECT_FALSE(path.has_value());
}

TEST(SearchTests, StartAndGoalNotInChunk)
{
    // Start is not in a chunk
    CostMap costMap;
    std::array<float,3> xyz_min = {1, 1, 1};
    std::array<float,3> xyz_max {3, 3, 3};
    costMap.addObstacle(xyz_min, xyz_max);
    std::array<float,3> start = {-1,-1,-1};
    std::array<float,3> goal = {4,4,4};
    auto path = Search::runSearch(costMap, start, goal);
    EXPECT_TRUE(path.has_value());

    // Goal is not in local region
    // goal = {20,20,20};
    // path = Search::runSearch(costMap, start, goal);
    // EXPECT_TRUE(path.has_value());
    // std::array<float,3> expected_local_goal = {16.5, 16.5, 16.5};
    // EXPECT_EQ(path.value().back(), expected_local_goal);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}