cmake_minimum_required(VERSION 3.8)
project(msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message files
  "msg/DetectedObject.msg"
  "msg/RobotStatus.msg"
  "msg/TrajectoryCommand.msg"
  "msg/TrajectoryPoint.msg"
  
  # Service files
  "srv/CaptureWorkspace.srv"
  "srv/SolveIK.srv"
  "srv/MoveTo.srv"

  # Action files
  "action/PickAndPlace.action"

  # Dependencies
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_package()
