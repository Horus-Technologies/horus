cmake_minimum_required(VERSION 3.5)
project(horus)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/home/misha/ros2_ws/install/drone_tracker_messages;/opt/ros/humble" ${CMAKE_PREFIX_PATH})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Include generated message headers
# include_directories(${drone_tracker_message_INCLUDE_DIRSs})

# Generate messages in the 'msg' folder of drone_tracker package
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Path.msg"  # Example message file in drone_tracker package
#   DEPENDENCIES drone_tracker_messages # Specify dependency on custom messages
# )

# Generate messages in the 'msg' folder
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Path.msg"
#   DEPENDENCIES std_msgs  # Add other dependencies as needed
# )

# Include Cpp "include" directory
include_directories(include)

add_executable(fsm 
  src/nodes/ssFSM.cpp
  src/nodes/ssGlobalPlanner.cpp
  src/nodes/ssLocalPlanner.cpp
  src/nodes/ssTrajectoryController.cpp  
  src/lib/Voxel.cpp
  src/lib/CostMap.cpp
  src/lib/Search.cpp
)
ament_target_dependencies(fsm
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  geographic_msgs
  Eigen3
)


# Create Cpp executable for local_planner
# add_executable(local_planner 
#   src/nodes/ssLocalPlanner.cpp 
#   src/lib/Voxel.cpp
#   src/lib/CostMap.cpp
#   src/lib/Search.cpp
# )
# ament_target_dependencies(local_planner
#   rclcpp
#   std_msgs
#   geometry_msgs
#   visualization_msgs
#   nav_msgs
#   geographic_msgs
#   Eigen3
# )

# find_package(planner_test REQUIRED)
# target_link_libraries(path_planner_exe planner_test)
# target_include_directories(path_planner_exe PUBLIC
#   ../../../../../planner_test/include
# )

# Create Cpp executable for trajectory_controller
# add_executable(trajectory_controller
#   src/nodes/ssTrajectoryController.cpp
# )
# ament_target_dependencies(trajectory_controller
#   rclcpp
#   std_msgs
#   geometry_msgs
#   geographic_msgs
#   Eigen3
#   nav_msgs
# )

add_executable(demoBFS
  src/nodes/demoBFS.cpp
  src/lib/Voxel.cpp
  src/lib/CostMap.cpp
  src/lib/Search.cpp
)
ament_target_dependencies(demoBFS
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  geographic_msgs
  Eigen3
)


find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(tests test/tests.cpp src/lib/Voxel.cpp src/lib/CostMap.cpp)
target_include_directories(tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install Cpp executables
install(TARGETS
  fsm
  # local_planner
  # trajectory_controller
  demoBFS
  tests
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# install(PROGRAMS
#   src/target_detector/TargetPosePublisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Add line to install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()