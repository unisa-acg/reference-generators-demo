cmake_minimum_required(VERSION 3.8)
project(gazebo_force_torque_sensor)

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

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    realtime_tools
    pluginlib
    gz_ros2_control
    ignition-gazebo6
    tf2
    tf2_eigen
    tf2_geometry_msgs
    acg_common_libraries
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})

# Shared library
add_library(gazebo_force_torque_sensor SHARED
            src/gazebo_ft_sensor.cpp
)

target_include_directories(
  gazebo_force_torque_sensor
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/gazebo_force_torque_sensor>
)

ament_target_dependencies(
  gazebo_force_torque_sensor
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export plugin
pluginlib_export_plugin_description_file(gz_ros2_control gazebo_force_torque_sensor.xml)

# Install
install(DIRECTORY include/gazebo_force_torque_sensor
        DESTINATION include/gazebo_force_torque_sensor
)

install(FILES gazebo_force_torque_sensor.xml
        DESTINATION share/gazebo_force_torque_sensor
)

install(TARGETS gazebo_force_torque_sensor
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

# exports
ament_export_include_directories(include)
ament_export_libraries(gazebo_force_torque_sensor)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(gazebo_force_torque_sensor)

ament_package()
