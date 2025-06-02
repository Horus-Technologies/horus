
#include "TrajectoryController.hpp"

using namespace std::chrono_literals;

TrajectoryController::TrajectoryController()
: Node("TrajectoryController"), _prev_desired_point(0,0,0), _prev_current_point(0,0,0)
{
    // Subscribing
    _subscriber_path = this->create_subscription<nav_msgs::msg::Path>(
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
  if (_path_avail){
    float lookahead_distance = 1;

    Eigen::Vector3f current_point
      (_current_pose.position.x,
      _current_pose.position.y,
      _current_pose.position.z);

    std::optional<Eigen::Vector3f> furthest_intersection = std::nullopt;
    
    // find furthest intersection from entire path
    for (int i = 0; i < _last_path->poses.size()-1; i++){
      geometry_msgs::msg::PoseStamped pose0 =  _last_path->poses[i];
      geometry_msgs::msg::PoseStamped pose1 =  _last_path->poses[i+1];
      Eigen::Vector3f point0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
      Eigen::Vector3f point1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
      
      std::vector<Eigen::Vector3f> intersections = Math::sphere_segment_intersection(point0, point1, current_point, lookahead_distance);
      if (intersections.size() == 2){
        // take intersection point closer to point1
        std::vector<float> distances_to_point1{};
        for (Eigen::Vector3f intersection : intersections){
          float distance = (intersection - point1).norm();
          distances_to_point1.push_back(distance);
        }
        auto min_iter = std::min_element(distances_to_point1.begin(), distances_to_point1.end());
        int min_index = std::distance(distances_to_point1.begin(),min_iter);
        furthest_intersection = intersections[min_index];
      }
      else if (intersections.size() == 1){
        furthest_intersection = intersections[0];
      }
    }

    Eigen::Vector3f desired_point;
    if(furthest_intersection.has_value()){
      desired_point = furthest_intersection.value();
    }
    else{
      // check if final endpoint is within the lookahead_distance already
      geometry_msgs::msg::PoseStamped pose_end =  _last_path->poses.back();
      Eigen::Vector3f point_end(pose_end.pose.position.x, pose_end.pose.position.y, pose_end.pose.position.z);
      if ((point_end - current_point).norm() <= lookahead_distance){
        desired_point = point_end;
      }
      else{
        // handle case of no intersection - in this case just go towards nearest projected point on path
        float minimum_project_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < _last_path->poses.size()-1; i++){
          geometry_msgs::msg::PoseStamped pose0 =  _last_path->poses[i];
          geometry_msgs::msg::PoseStamped pose1 =  _last_path->poses[i+1];
          Eigen::Vector3f point0(pose0.pose.position.x, pose0.pose.position.y, pose0.pose.position.z);
          Eigen::Vector3f point1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
  
          Eigen::Vector3f projected_point = Math::point_segment_projection(current_point, point0, point1);
          float distance = (current_point - projected_point).norm();
          if (distance < minimum_project_distance){
            minimum_project_distance = distance;
            Eigen::Vector3f dir = (projected_point - current_point)/(projected_point - current_point).norm();
            desired_point = current_point + dir * lookahead_distance;
          }
        }
      }
    }

      // Publish desired pose for Rviz
      auto desired_pose_stamped = geometry_msgs::msg::PoseStamped();
      desired_pose_stamped.header.frame_id = "odom";
      desired_pose_stamped.header.stamp = this->now();
      desired_pose_stamped.pose.position.x = desired_point[0];
      desired_pose_stamped.pose.position.y = desired_point[1];
      desired_pose_stamped.pose.position.z = desired_point[2];
      _publisher_desired->publish(desired_pose_stamped);

      //PID Algo
      Eigen::Vector3f k_p(1,1,1);
      Eigen::Vector3f k_d(0.5,0.5,0.5);
      float k_p_yaw = 0.75;

      Eigen::Vector3f d_desired_point = desired_point - _prev_desired_point;
      Eigen::Vector3f d_current_point = current_point - _prev_current_point;

      Eigen::Vector3f command_linear_world = k_p.cwiseProduct(desired_point - current_point)
        + k_d.cwiseProduct(d_desired_point - d_current_point);

      // Rotate Twist command into drone frame
      Eigen::Quaternionf current_quat(_current_pose.orientation.w,
                                    _current_pose.orientation.x,
                                    _current_pose.orientation.y,
                                    _current_pose.orientation.z);
      current_quat.normalize(); // Make quaternion unit for conversion to rotation matrix.

      Eigen::Matrix3f current_rot = current_quat.toRotationMatrix();

      Eigen::Vector3f command_linear_drone = current_rot.transpose() * command_linear_world;
      
      geometry_msgs::msg::Twist command_twist;
      command_twist.linear.x = command_linear_drone[0];
      command_twist.linear.y = -command_linear_drone[1];
      command_twist.linear.z = command_linear_drone[2];

      // Orientation Control
      float desired_yaw;
      if (_last_path->poses.size() > 1){
        Eigen::Vector3f vec = desired_point - current_point;
        desired_yaw = atan2(-vec(0),vec(1));
      }
      else{
        desired_yaw = _prev_desired_yaw;
      }
      Eigen::Vector3f ref(1,0,0);
      Eigen::Vector3f drone_vec = current_rot * ref;
      
      float current_yaw = atan2(-drone_vec(0),drone_vec(1));

      const double pi = std::atan(1.0)*4;
      float error_yaw = desired_yaw - current_yaw;
      if (error_yaw > pi){
        error_yaw = error_yaw - 2*pi;
      }
      else if (error_yaw < -pi){
        error_yaw = error_yaw + 2*pi;
      }
      float command_yaw = k_p_yaw * error_yaw;
      command_twist.angular.z = command_yaw;

      geometry_msgs::msg::TwistStamped command_twist_stamped;
      command_twist_stamped.header.frame_id = "base_link";
      command_twist_stamped.header.stamp = this->now();
      command_twist_stamped.twist = command_twist;
      _publisher->publish(command_twist_stamped);

      _prev_desired_point = desired_point;
      _prev_current_point = current_point;
      _prev_desired_yaw = desired_yaw;

      // RCLCPP_INFO(this->get_logger(), "Desired Point: [%f, %f, %f]", desired_point[0],desired_point[1],desired_point[2]);
      // RCLCPP_INFO(this->get_logger(), "Command Linear: [%f, %f, %f]", command_twist.linear.x,command_twist.linear.y,command_twist.linear.z);
      // RCLCPP_INFO(this->get_logger(), "Command Yaw: %f", command_twist.angular.z);
  }
}

void TrajectoryController::callback_drone(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamp)
{
  _current_pose = pose_stamp->pose;
}


void TrajectoryController::callback_path(const nav_msgs::msg::Path::SharedPtr path)
{
  _last_path = path;
  _path_avail = true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryController>());
  rclcpp::shutdown();
  return 0;
}
