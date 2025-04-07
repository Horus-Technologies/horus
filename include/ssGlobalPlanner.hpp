
#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "CostMap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstdlib>

class ssGlobalPlanner : public rclcpp::Node
{
    public:
        ssGlobalPlanner(CostMap* costMap);

    private:
        void run();
        void run_random();
        void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp);

        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _publisherGoal;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriberDrone;
        geometry_msgs::msg::PoseStamped _lastPoseDrone;

        CostMap* _costMap;
        bool _planComplete = false;
        const Voxel* _start; //currently unused, but may need later
        std::vector<std::vector<uint16_t>> _goals;
        int _currentGoalIndex;
};

#endif // GLOBAL_PLANNER_H