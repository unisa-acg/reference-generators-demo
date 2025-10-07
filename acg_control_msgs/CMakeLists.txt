cmake_minimum_required(VERSION 3.8)
project(acg_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/FollowJointTrajectory.action"
  "action/FollowTaskSpaceTrajectory.action"
  "msg/AdmittanceControllerState.msg"
  "msg/JointTrajectory.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/JointWrenchPoint.msg"
  "msg/TaskSpaceTrajectory.msg"
  "msg/TaskSpaceTrajectoryPoint.msg"
  "msg/TaskSpacePoint.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  control_msgs
)
ament_export_dependencies(
  rosidl_default_runtime
  control_msgs
  std_msgs
  geometry_msgs
)

ament_package()
