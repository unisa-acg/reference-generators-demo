cmake_minimum_required(VERSION 3.8)
project(gravity_compensation_filter)

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

# Find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    filters
    pluginlib
    acg_common_libraries
    urdf
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Shared library
add_library(gravity_compensation_filter SHARED
            src/gravity_compensation_filter.cpp
)

target_include_directories(
  gravity_compensation_filter
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/gravity_compensation_filter>
)

ament_target_dependencies(gravity_compensation_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(filters gravity_compensation_filter.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  gravity_compensation_filter
  PRIVATE "GRAVITY_COMPENSATION_FILTER_BUILDING_LIBRARY"
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES gravity_compensation_filter.xml
        DESTINATION share/gravity_compensation_filter
)

install(TARGETS gravity_compensation_filter
        EXPORT export_${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# Tests
if(BUILD_TESTING)
  # Find dependencies
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  set(THIS_TEST_PACKAGE_INCLUDE_DEPENDS
      rclcpp
      filters
      geometry_msgs
      trajectory_msgs
      tf2_geometry_msgs
      tf2
      tf2_ros
      rosbag2_cpp
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    if(NOT ${dependency}_FOUND)
      find_package(${dependency} REQUIRED)
    endif()
  endforeach()

  ament_add_gtest_executable(gravity_compensation_variable_orientation_test
                             test/test_gravity_compensation_variable_orientation.cpp
  )

  add_ros_test(test/test_gravity_compensation_variable_orientation.py
               TIMEOUT
               600
  )

  ament_target_dependencies(gravity_compensation_variable_orientation_test
                            ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )

  install(DIRECTORY launch
          DESTINATION share/${PROJECT_NAME}/
  )

  # Install test files
  install(TARGETS gravity_compensation_variable_orientation_test
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY test/bagfiles test/config
          DESTINATION share/${PROJECT_NAME}/test
  )

  ament_export_dependencies(${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
