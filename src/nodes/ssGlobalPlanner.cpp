/*
Publishes a goal waypoint the drone must go to.
This node has access to the costMap.
*/

#include "ssGlobalPlanner.hpp"

using namespace std::chrono_literals;

ssGlobalPlanner::ssGlobalPlanner(CostMap* costMap) 
: Node("ssGlobalPlanner"), _costMap(costMap), _currentGoalIndex(0)
{
    // Subscribing
    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriberDrone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ap/pose/filtered", qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp) {
        this->callback_drone(poseStamp);
    });

    _publisherGoal = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/global_goal", 10);
    _timer = this->create_wall_timer(20ms, std::bind(&ssGlobalPlanner::run_random, this));

    //goal indices need to be within costmap!!
    // _goals.push_back({20,70,8});
    // _goals.push_back({50,60,8});
    // _goals.push_back({1,1,5});

    // _goals.push_back({79,5,5});
    // _goals.push_back({1,15,15});
    // _goals.push_back({1,1,1});

    _goals.push_back({1,1,6});
}

void ssGlobalPlanner::run()
{
    if(!_planComplete){
        // Check if current goal is reached, and update _currentGoalIndex if so
        std::array<int,3> goalPointVoxel = {
            _goals[_currentGoalIndex][0],
            _goals[_currentGoalIndex][1],
            _goals[_currentGoalIndex][2]};
        
        float distanceToGoalPoint = sqrt(
            pow(_lastPoseDrone.pose.position.x - _costMap->getVoxelPosition(goalPointVoxel)[0], 2) +
            pow(_lastPoseDrone.pose.position.y - _costMap->getVoxelPosition(goalPointVoxel)[1], 2) +
            pow(_lastPoseDrone.pose.position.z - _costMap->getVoxelPosition(goalPointVoxel)[2], 2));
        if (distanceToGoalPoint < 0.05)
        {   
            _currentGoalIndex = _currentGoalIndex + 1;
            if(_currentGoalIndex == _goals.size())
            {
                // _planComplete = true;
                _currentGoalIndex = 0;
            }
        }
        std_msgs::msg::UInt16MultiArray goal;
        std::vector<uint16_t> goalData = _goals[_currentGoalIndex];
        goal.data = goalData;
        _publisherGoal->publish(goal);
    }
}

void ssGlobalPlanner::run_random()
{
    std::array<int,3> goalPointVoxel = {_goals[0][0], _goals[0][1], _goals[0][2]};

    float distanceToGoalPoint = sqrt(
        pow(_lastPoseDrone.pose.position.x - _costMap->getVoxelPosition(goalPointVoxel)[0], 2) +
        pow(_lastPoseDrone.pose.position.y - _costMap->getVoxelPosition(goalPointVoxel)[1], 2) +
        pow(_lastPoseDrone.pose.position.z - _costMap->getVoxelPosition(goalPointVoxel)[2], 2));
    if (distanceToGoalPoint < 0.05 || _costMap->getVoxelStateByIndices(goalPointVoxel) == VoxelState::OCCUPIED)
    {   
        _goals[0] = {rand() % 40, rand() % 80, rand() % 15};
        goalPointVoxel = {_goals[0][0], _goals[0][1], _goals[0][2]};
        while(_costMap->getVoxelStateByIndices(goalPointVoxel) == VoxelState::OCCUPIED){
            _goals[0] = {rand() % 80, rand() % 80, rand() % 15};
            goalPointVoxel = {_goals[0][0], _goals[0][1], _goals[0][2]};
        } 
    }
    std_msgs::msg::UInt16MultiArray goal;
    std::vector<uint16_t> goalData = _goals[_currentGoalIndex];
    goal.data = goalData;
    _publisherGoal->publish(goal);
    RCLCPP_INFO(this->get_logger(),"Goal sent: %d %d %d", 
    _goals[0][0],
    _goals[0][1],
    _goals[0][2]);
}

void ssGlobalPlanner::callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp)
{
  _lastPoseDrone = *poseStamp; // odom frame
}