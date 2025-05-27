
#include "TrajectoryController.hpp"

using namespace std::chrono_literals;

TrajectoryController::TrajectoryController()
: Node("TrajectoryController"), _count(0), _prev_desired_pose(0,0,0), _prev_current_pose(0,0,0)
{
    // Subscribing
    _subscriber = this->create_subscription<nav_msgs::msg::Path>(
        "/local_plan/clean_path",10,std::bind(&TrajectoryController::callback_path,this,std::placeholders::_1));
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
      float speed_multiplier = 1;
      double dt = 0.01;

      geometry_msgs::msg::PoseStamped pose0 =  _last_path->poses[_current_pose_index-1];
      geometry_msgs::msg::PoseStamped pose1 =  _last_path->poses[_current_pose_index];
      Eigen::Vector3d P0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
      Eigen::Vector3d P1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
      Eigen::Vector3d vec = P1 - P0;

      float segment_duration = vec.norm(); //ms

      std::cout << "segment duration: " << segment_duration << std::endl;
      s = s + dt * speed_multiplier/segment_duration; // need to normalize by segment duration/distance
      // reaching s=1 means need to select next pose as target or signal path completed
      if (s>=1.0){
        if (_current_pose_index == _last_path->poses.size()-1){
          s=1;
        }
        else{ // switch to next segment of path
          _current_pose_index = _current_pose_index + 1;
          _time_start_follow = now;
          std::cout << "switch to next segment" << std::endl;
          s = 0;
        }
      }
      
      std::cout << "s: " << s <<std::endl;
      
  
      Eigen::Vector3d desired_pose_vec = P0 + s*vec;              

      
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

  
  std::cout << " " << std::endl;
  for (auto p : path->poses){
    RCLCPP_INFO(this->get_logger(), "Received pose: [%f, %f, %f]",
      p.pose.position.x,
      p.pose.position.y,
      p.pose.position.z);
  }

  // compute s parameter offset representing current pure pursuit control effort so that path switch is smooth
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

  float ab = (b-a).norm();
  float ac = 0;
  ac = ac + (_prev_desired_pose - _prev_current_pose).norm(); // add current pure pursuit error to offset to maintain speed during path change
  s = ac / ab;
  if (s < 0){s = 0;}

  RCLCPP_INFO(this->get_logger(), "s right after projection: %f",s);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryController>());
  rclcpp::shutdown();
  return 0;
}
