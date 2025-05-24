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
    size_t _count;

    // Subscriber members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriber_drone;
    std::shared_ptr<nav_msgs::msg::Path> _last_path{};
    geometry_msgs::msg::Pose _current_pose_drone;

    // General members
    double s = 0;
    double _time_start_follow;
    bool _path_started = false;
    bool _path_complete = false;
    bool _path_avail = false;
    int _current_pose_index; // index of the current pose drone is heading towards
    float _s_offset;

    Eigen::Vector3d _prev_desired_pose;
    Eigen::Vector3d _prev_current_pose;
    float _prev_desired_yaw;
};
