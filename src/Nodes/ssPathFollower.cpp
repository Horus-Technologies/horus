/*
This file contains PID loop to follow local path plan. It
- Subscribe to 'path' topic from path_planner and current drone position
- Compute desired position point for drone (in drone frame)
- Publish this to topic 'position_desired', which gets accessed by Gazebo's controller
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
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class ssPathFollower : public rclcpp::Node
{
  public:
    ssPathFollower()
    : Node("PathFollower"), _count(0), _s_offset(0.0), prevDesiredPose(0,0,0), prevCurrentPose(0,0,0)
    {
        // Subscribing
        _subscriber = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints",10,std::bind(&ssPathFollower::callback_path,this,std::placeholders::_1));
        rclcpp::QoS qos(rclcpp::KeepLast(10)); 
        qos.best_effort();
       _subscriberDrone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered", qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr geoPoseStamp) {
            this->callback_drone(geoPoseStamp);
        });

        // Publishing
        _publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", qos);
        _timer = this->create_wall_timer(
        10ms, std::bind(&ssPathFollower::callback_command, this));

        _publisherDesired = this->create_publisher<geometry_msgs::msg::PoseStamped>("desired_pose",10);
        
    }

  private:
    // Publisher function for path - on a timer callback
    void callback_command()
    { 
      // auto now = std::chrono::steady_clock::now();
      // auto duration = now.time_since_epoch();
      // auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

      if (_pathAvail){
          // Get relative time from start of following
          float speedMultiplier = 1;
          double now = _count * 0.01; // sec
          double t = (now - _timeStartFollow)*speedMultiplier; // Seconds
          float segmentDuration = 0; //ms
          double s = 0;
          // segmentDuration = _lastPath->poses[_currentPoseIndex].header.stamp.sec
          //   - _lastPath->poses[_currentPoseIndex-1].header.stamp.sec;
          RCLCPP_INFO(this->get_logger(), "Path size: %ld", _lastPath->poses.size());
          if (_lastPath->poses.size() > 1){
            segmentDuration = _lastPath->poses[_currentPoseIndex].header.stamp.sec
            - _lastPath->poses[_currentPoseIndex-1].header.stamp.sec;
            if (_currentPoseIndex == 1){s = s + _s_offset;} 
          }
          else{
            // situation where path is composed only of the final pose
            segmentDuration = 3000;
          }
          RCLCPP_INFO(this->get_logger(), "segmentDuration = %f", segmentDuration/1000);
          s = s + t/(segmentDuration/1000);
          // if (s < 0){s=0;}
          
          // reaching s=1 means need to select next pose as target or signal path completed
          if (s>=1.0){
            if (_currentPoseIndex == _lastPath->poses.size()-1){
              s=1;
            }
            else{ // switch to next segment of path
              _currentPoseIndex = _currentPoseIndex + 1;
              _timeStartFollow = now;
              s = 0;
            }
          }
          RCLCPP_INFO(this->get_logger(), "s = %f",s);
          Eigen::Vector3d vec;
          Eigen::Vector3d desiredPose_vec;
          if (_lastPath->poses.size() > 1){
            geometry_msgs::msg::PoseStamped pose0 =  _lastPath->poses[_currentPoseIndex-1];
            geometry_msgs::msg::PoseStamped pose1 =  _lastPath->poses[_currentPoseIndex];
            Eigen::Vector3d P0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
            Eigen::Vector3d P1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
            vec = P1-P0;
            desiredPose_vec = P0 + s*vec;              
          }
          else{
            geometry_msgs::msg::PoseStamped pose = _lastPath->poses[0]; 
            desiredPose_vec(0) = pose.pose.position.x;         
            desiredPose_vec(1) = pose.pose.position.y;   
            desiredPose_vec(2) = pose.pose.position.z;   
          }
          RCLCPP_INFO(this->get_logger(), "Desired Pose: [%f, %f, %f]",
          desiredPose_vec[0],
          desiredPose_vec[1],
          desiredPose_vec[2]);
          
          Eigen::Vector3d currentPoseDrone_vec
          (_currentPoseDrone.position.x,
          _currentPoseDrone.position.y,
          _currentPoseDrone.position.z);

          RCLCPP_INFO(this->get_logger(), "Current Pose: [%f, %f, %f]",
          currentPoseDrone_vec[0],
          currentPoseDrone_vec[1],
          currentPoseDrone_vec[2]);

          // Publish desired pose for Rviz
          auto desiredPoseStamped = geometry_msgs::msg::PoseStamped();
          desiredPoseStamped.header.frame_id = "odom";
          desiredPoseStamped.header.stamp = this->now();
          desiredPoseStamped.pose.position.x = desiredPose_vec[0];
          desiredPoseStamped.pose.position.y = desiredPose_vec[1];
          desiredPoseStamped.pose.position.z = desiredPose_vec[2];
          _publisherDesired->publish(desiredPoseStamped);

          //PID Algo
          Eigen::Vector3d Kp(1,1,1);
          Eigen::Vector3d Kd(0.5,0.5,0.5);
          float Kp_yaw = 0.5;

          Eigen::Vector3d d_desiredPose = desiredPose_vec - prevDesiredPose;
          Eigen::Vector3d d_currentPose = currentPoseDrone_vec - prevCurrentPose;

          Eigen::Vector3d commandLinear_world = Kp.cwiseProduct(desiredPose_vec - currentPoseDrone_vec)
            + Kd.cwiseProduct(d_desiredPose - d_currentPose);

          // Rotate Twist command into drone frame
          Eigen::Quaterniond drone_quat(_currentPoseDrone.orientation.w,
                                        _currentPoseDrone.orientation.x,
                                        _currentPoseDrone.orientation.y,
                                        _currentPoseDrone.orientation.z);
          drone_quat.normalize(); // Make quaternion unit for conversion to rotation matrix.

          Eigen::Matrix3d drone_rotmat = drone_quat.toRotationMatrix();

          Eigen::Vector3d commandLinear_drone = drone_rotmat.transpose() * commandLinear_world;
          
          geometry_msgs::msg::Twist commandTwist;
          commandTwist.linear.x = commandLinear_drone[0];
          commandTwist.linear.y = -commandLinear_drone[1];
          commandTwist.linear.z = commandLinear_drone[2];

          // Orientation Control
          float desiredYaw;
          if (_lastPath->poses.size() > 1){
            desiredYaw = atan2(-vec(0),vec(1));
          }
          else{
            desiredYaw = prevDesiredYaw;
          }
          RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f",desiredYaw);
          Eigen::Vector3d ref(1,0,0);
          Eigen::Vector3d drone_vec = drone_rotmat * ref;
          
          float currentYaw = atan2(-drone_vec(0),drone_vec(1));
          RCLCPP_INFO(this->get_logger(), "Current Yaw: %f",currentYaw);
          const double pi = std::atan(1.0)*4;
          float errorYaw = desiredYaw - currentYaw;
          if (errorYaw > pi){
            errorYaw = errorYaw - 2*pi;
          }
          else if (errorYaw < -pi){
            errorYaw = errorYaw + 2*pi;
          }
          float commandYaw = Kp_yaw * errorYaw;
          commandTwist.angular.z = commandYaw;
          // RCLCPP_INFO(this->get_logger(), "Publishing Twist Command:\nPosition: [%f, %f, %f]",
          //             commandTwist.linear.x, commandTwist.linear.y, commandTwist.linear.z);

          geometry_msgs::msg::TwistStamped commandTwistStamped;
          commandTwistStamped.header.frame_id = "base_link";
          commandTwistStamped.header.stamp = this->now();
          commandTwistStamped.twist = commandTwist;
          _publisher->publish(commandTwistStamped);

          prevDesiredPose = desiredPose_vec;
          prevCurrentPose = currentPoseDrone_vec;
          prevDesiredYaw = desiredYaw;
          // _count++;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "No Path Avail Yet");
      }
      _count++;
    }

    Eigen::Vector3d getBezierPoint(std::vector<Eigen::Vector3d>& points, double s) {
      if (points.size() <= 0) {
        RCLCPP_INFO(this->get_logger(), "Num Ponts = 0");
        return Eigen::Vector3d::Zero(); // Handle empty input
      }
  
      std::vector<Eigen::Vector3d> tmp = points; // Use std::vector, initialize with points
  
      int i = points.size() - 1;
      while (i > 0) {
          for (int k = 0; k < i; k++) {
              tmp[k] = (1.0 - s) * tmp[k] + s * tmp[k + 1]; // Correct De Casteljau's algorithm
          }
          i--;
      }
      return tmp[0];
    }

    void callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr poseStamp)
    {
      _currentPoseDrone = poseStamp->pose;
    }
    

    void callback_path(const nav_msgs::msg::Path::SharedPtr path)
    {
      // auto now = std::chrono::steady_clock::now();
      // auto duration = now.time_since_epoch();
      // auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
      float now = _count * 0.01; // sec
      _lastPath = path;
      _pathAvail = true;
      // Get reference time
      _timeStartFollow = now; // Seconds
      _pathStarted = true;
      _currentPoseIndex = 1; // start going towards the next pose after drone start

      // compute s parameter offset
      Eigen::Vector3f A({
        _lastPath->poses[0].pose.position.x,
        _lastPath->poses[0].pose.position.y,
        _lastPath->poses[0].pose.position.z});
      Eigen::Vector3f B({
        _lastPath->poses[1].pose.position.x,
        _lastPath->poses[1].pose.position.y,
        _lastPath->poses[1].pose.position.z});
      Eigen::Vector3f C({
        _currentPoseDrone.position.x,
        _currentPoseDrone.position.y,
        _currentPoseDrone.position.z});
        
      if (path->poses.size() > 1){
        float AB = (B-A).norm();
        float AC = (C-A).dot(B-A) / AB; // C projected onto AB
        // if(AC<0){AC=0;}
        AC = AC + (prevDesiredPose - prevCurrentPose).norm(); // add current pure pursuit error to offset to maintain speed during path change
        _s_offset = AC / AB;
        if (_s_offset < 0){_s_offset = 0;}
      }
      else{
        _s_offset = 0.0;
      }
      
    }

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ssPathFollower>());
  rclcpp::shutdown();
  return 0;
}