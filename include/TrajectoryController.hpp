/*
This file contains a PID loop to follow local path plan, traversing through
all local waypoints. Whenever a new path plan arrives, it restarts and follows
the new plan.
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Math.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "nav_msgs/msg/path.hpp"

class TrajectoryController : public rclcpp::Node
{
  public:
    TrajectoryController();

  private:
    void callback_command();
    void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp);
    void callback_path(const nav_msgs::msg::Path::SharedPtr path);

    // Publisher members
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer_desired;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _publisher_desired;
    
    // Subscriber members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _subscriber_path;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriber_drone;
    std::shared_ptr<nav_msgs::msg::Path> _last_path{};
    geometry_msgs::msg::Pose _current_pose;
    
    // General members
    bool _path_avail = false;
    Eigen::Vector3f _prev_desired_point;
    Eigen::Vector3f _prev_current_point;
    float _prev_desired_yaw;
};
