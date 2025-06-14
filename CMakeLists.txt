cmake_minimum_required(VERSION 3.5)
project(horus)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-g)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Include Cpp "include" directory
include_directories(include)

add_executable(fsm 
  src/nodes/FSM.cpp
  src/nodes/motion_control/GlobalPlanner.cpp
  src/nodes/motion_control/LocalPlanner.cpp
  src/nodes/perception/VoxelMapper.cpp  
  src/lib/VoxelGrid.cpp
  src/lib/Search.cpp
  src/lib/Chunk.cpp
)
ament_target_dependencies(fsm
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  Eigen3
  tf2_ros
)

# Create Cpp executable for trajectory_controller
add_executable(trajectory_controller
  src/nodes/motion_control/TrajectoryController.cpp
  src/lib/Math.cpp
)
ament_target_dependencies(trajectory_controller
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
  nav_msgs
)

add_executable(demo_voxel_mapper
  src/nodes/perception/DemoVoxelMapper.cpp
  src/nodes/perception/VoxelMapper.cpp
  src/lib/VoxelGrid.cpp
  src/lib/Chunk.cpp
)
ament_target_dependencies(demo_voxel_mapper
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  Eigen3
)

# Tests
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(tests 
  test/tests.cpp 
  src/lib/Chunk.cpp 
  src/lib/VoxelGrid.cpp
  src/lib/Search.cpp
  src/lib/Math.cpp)
target_include_directories(tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install Cpp executables
install(TARGETS
  fsm
  trajectory_controller
  demo_voxel_mapper
  tests
  DESTINATION lib/${PROJECT_NAME}
)

# Add line to install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
      DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

ament_package()