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
// #include "geographic_msgs/msg/geo_pose_stamped.hpp"
// #include "geographic_msgs/msg/geo_pose.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "nav_msgs/msg/path.hpp"

class ssTrajectoryController : public rclcpp::Node
{
  public:
    ssTrajectoryController();

  private:
    void callback_command();
    void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp);
    void callback_path(const nav_msgs::msg::Path::SharedPtr path);

    // Publisher members
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timerDesired;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _publisherDesired;
    size_t _count;

    // Subscriber members
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _subscriberDrone;
    std::shared_ptr<nav_msgs::msg::Path> _lastPath{};
    geometry_msgs::msg::Pose _currentPoseDrone;

    // General members
    double _timeStartFollow;
    bool _pathStarted = false;
    bool _pathComplete = false;
    bool _pathAvail = false;
    int _currentPoseIndex; // index of the current pose drone is heading towards
    float _s_offset;

    Eigen::Vector3d prevDesiredPose;
    Eigen::Vector3d prevCurrentPose;
    float prevDesiredYaw;
};
