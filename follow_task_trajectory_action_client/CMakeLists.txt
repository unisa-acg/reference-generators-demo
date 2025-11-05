cmake_minimum_required(VERSION 3.8)
project(follow_task_trajectory_action_client)

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
set(THIS_PACKAGE_INCLUDE_DEPENDS
    acg_control_msgs
    rclcpp_action
    rclcpp_components
    rclcpp
    ament_index_cpp
    rosbag2_cpp
)
find_package(ament_cmake REQUIRED)
foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(task_trajectory_action_client SHARED
            src/follow_task_trajectory_action_client.cpp
)

target_include_directories(
  task_trajectory_action_client
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

target_compile_features(
  task_trajectory_action_client
  PUBLIC c_std_99 cxx_std_17
) # Require C99 and C++17

target_link_libraries(task_trajectory_action_client stdc++fs)

target_compile_definitions(
  task_trajectory_action_client
  PRIVATE "FOLLOW_TASK_TRAJECTORY_ACTION_CLIENT_CPP_BUILDING_DLL"
)

ament_target_dependencies(
  task_trajectory_action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "ament_index_cpp"
  "rosbag2_cpp"
  "acg_control_msgs"
)

rclcpp_components_register_node(
  task_trajectory_action_client
  PLUGIN
  "follow_task_trajectory_action_client_cpp::FollowTaskTrajectoryActionClient"
  EXECUTABLE
  follow_task_trajectory_action_client
)

install(TARGETS task_trajectory_action_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch trajectories
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
