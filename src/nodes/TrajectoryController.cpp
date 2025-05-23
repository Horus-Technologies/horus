
#include "TrajectoryController.hpp"

using namespace std::chrono_literals;

TrajectoryController::TrajectoryController()
: Node("TrajectoryController"), _count(0), _s_offset(0.0), _prev_desired_pose(0,0,0), _prev_current_pose(0,0,0)
{
    // Subscribing
    _subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "waypoints",10,std::bind(&TrajectoryController::callback_path,this,std::placeholders::_1));
    rclcpp::QoS qos(rclcpp::KeepLast(10)); 
    qos.best_effort();
    _subscriber_drone = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ap/pose/filtered", qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp) {
        this->callback_drone(pose_stamp);
    });

    // Publishing
    _publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/cmd_vel", qos);
    _timer = this->create_wall_timer(
    10ms, std::bind(&TrajectoryController::callback_command, this));

    _publisher_desired = this->create_publisher<geometry_msgs::msg::PoseStamped>("desired_pose",10);
}

// Publisher function for path - on a timer callback
void TrajectoryController::callback_command()
{ 
  double now = _count * 0.01; // sec
  if (_path_avail){
      // Get relative time from start of following
      float speed_multiplier = 0.75;
      
      double t = (now - _time_start_follow)*speed_multiplier; // sec
      float segment_duration = 0; //ms
      double s = 0;

      if (_last_path->poses.size() > 1){
        segment_duration = _last_path->poses[_current_pose_index].header.stamp.sec
        - _last_path->poses[_current_pose_index-1].header.stamp.sec;
        if (_current_pose_index == 1){s = s + _s_offset;} 
      }
      else{
        // situation where path is composed only of the final pose
        segment_duration = 3000;
      }
      
      s = s + t/(segment_duration/1000);
      
      // reaching s=1 means need to select next pose as target or signal path completed
      if (s>=1.0){
        if (_current_pose_index == _last_path->poses.size()-1){
          s=1;
        }
        else{ // switch to next segment of path
          _current_pose_index = _current_pose_index + 1;
          _time_start_follow = now;
          s = 0;
        }
      }
      
      Eigen::Vector3d vec;
      Eigen::Vector3d desired_pose_vec;
      if (_last_path->poses.size() > 1){
        geometry_msgs::msg::PoseStamped pose0 =  _last_path->poses[_current_pose_index-1];
        geometry_msgs::msg::PoseStamped pose1 =  _last_path->poses[_current_pose_index];
        Eigen::Vector3d P0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
        Eigen::Vector3d P1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
        vec = P1-P0;
        desired_pose_vec = P0 + s*vec;              
      }
      else{
        geometry_msgs::msg::PoseStamped pose = _last_path->poses[0]; 
        desired_pose_vec(0) = pose.pose.position.x;         
        desired_pose_vec(1) = pose.pose.position.y;   
        desired_pose_vec(2) = pose.pose.position.z;   
      }
      
      Eigen::Vector3d current_pose_drone_vec
      (_current_pose_drone.position.x,
      _current_pose_drone.position.y,
      _current_pose_drone.position.z);

      // Publish desired pose for Rviz
      auto desired_pose_stamped = geometry_msgs::msg::PoseStamped();
      desired_pose_stamped.header.frame_id = "odom";
      desired_pose_stamped.header.stamp = this->now();
      desired_pose_stamped.pose.position.x = desired_pose_vec[0];
      desired_pose_stamped.pose.position.y = desired_pose_vec[1];
      desired_pose_stamped.pose.position.z = desired_pose_vec[2];
      _publisher_desired->publish(desired_pose_stamped);

      //PID Algo
      Eigen::Vector3d k_p(1,1,1);
      Eigen::Vector3d k_d(0.5,0.5,0.5);
      float k_p_yaw = 0.75;

      Eigen::Vector3d d_desired_pose = desired_pose_vec - _prev_desired_pose;
      Eigen::Vector3d d_current_pose = current_pose_drone_vec - _prev_current_pose;

      Eigen::Vector3d command_linear_world = k_p.cwiseProduct(desired_pose_vec - current_pose_drone_vec)
        + k_d.cwiseProduct(d_desired_pose - d_current_pose);

      // Rotate Twist command into drone frame
      Eigen::Quaterniond drone_quat(_current_pose_drone.orientation.w,
                                    _current_pose_drone.orientation.x,
                                    _current_pose_drone.orientation.y,
                                    _current_pose_drone.orientation.z);
      drone_quat.normalize(); // Make quaternion unit for conversion to rotation matrix.

      Eigen::Matrix3d drone_rot_mat = drone_quat.toRotationMatrix();

      Eigen::Vector3d command_linear_drone = drone_rot_mat.transpose() * command_linear_world;
      
      geometry_msgs::msg::Twist command_twist;
      command_twist.linear.x = command_linear_drone[0];
      command_twist.linear.y = -command_linear_drone[1];
      command_twist.linear.z = command_linear_drone[2];

      // Orientation Control
      float desired_yaw;
      if (_last_path->poses.size() > 1){
        desired_yaw = atan2(-vec(0),vec(1));
      }
      else{
        desired_yaw = _prev_desired_yaw;
      }
      Eigen::Vector3d ref(1,0,0);
      Eigen::Vector3d drone_vec = drone_rot_mat * ref;
      
      float current_yaw = atan2(-drone_vec(0),drone_vec(1));

      const double pi = std::atan(1.0)*4;
      float error_yaw = desired_yaw - current_yaw;
      if (error_yaw > pi){
        error_yaw = error_yaw - 2*pi;
      }
      else if (error_yaw < -pi){
        error_yaw = error_yaw + 2*pi;
      }
      float command_Yaw = k_p_yaw * error_yaw;
      command_twist.angular.z = command_Yaw;

      geometry_msgs::msg::TwistStamped command_twist_stamped;
      command_twist_stamped.header.frame_id = "base_link";
      command_twist_stamped.header.stamp = this->now();
      command_twist_stamped.twist = command_twist;
      _publisher->publish(command_twist_stamped);

      _prev_desired_pose = desired_pose_vec;
      _prev_current_pose = current_pose_drone_vec;
      _prev_desired_yaw = desired_yaw;

      // RCLCPP_INFO(this->get_logger(), "segment_duration = %f", segment_duration/1000);
      // RCLCPP_INFO(this->get_logger(), "s = %f",s);
      // RCLCPP_INFO(this->get_logger(), "Desired Pose: [%f, %f, %f]",
      // desired_pose_vec[0],
      // desired_pose_vec[1],
      // desired_pose_vec[2]);
      // RCLCPP_INFO(this->get_logger(), "Current Pose: [%f, %f, %f]",
      // current_pose_drone_vec[0],
      // current_pose_drone_vec[1],
      // current_pose_drone_vec[2]);
      // RCLCPP_INFO(this->get_logger(), "Current Yaw: %f",current_yaw);
      // RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f",desired_yaw);
  }

  _count++;
}

void TrajectoryController::callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp)
{
  _current_pose_drone = pose_stamp->pose;
}


void TrajectoryController::callback_path(const nav_msgs::msg::Path::SharedPtr path)
{
  float now = _count * 0.01; // sec
  _last_path = path;
  _path_avail = true;
  // Get reference time
  _time_start_follow = now; // sec
  _path_started = true;
  _current_pose_index = 1; // start going towards the next pose after drone start

  // compute s parameter offset
  Eigen::Vector3f a({
    _last_path->poses[0].pose.position.x,
    _last_path->poses[0].pose.position.y,
    _last_path->poses[0].pose.position.z});
  Eigen::Vector3f b({
    _last_path->poses[1].pose.position.x,
    _last_path->poses[1].pose.position.y,
    _last_path->poses[1].pose.position.z});
  Eigen::Vector3f c({
    _current_pose_drone.position.x,
    _current_pose_drone.position.y,
    _current_pose_drone.position.z});
    
  if (path->poses.size() > 1){
    float ab = (b-a).norm();
    float ac = (c-a).dot(b-a) / ab; // C projected onto AB
    ac = ac + (_prev_desired_pose - _prev_current_pose).norm(); // add current pure pursuit error to offset to maintain speed during path change
    _s_offset = ac / ab;
    if (_s_offset < 0){_s_offset = 0;}
  }
  else{
    _s_offset = 0.0;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryController>());
  rclcpp::shutdown();
  return 0;
}
