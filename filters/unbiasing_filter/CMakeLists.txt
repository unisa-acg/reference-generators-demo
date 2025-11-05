cmake_minimum_required(VERSION 3.8)
project(unbiasing_filter)

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
    multi_channel_moving_average_filter
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(unbiasing_filter SHARED
            src/unbiasing_filter.cpp
)

target_include_directories(
  unbiasing_filter
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/unbiasing_filter>
)

ament_target_dependencies(unbiasing_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(filters unbiasing_filter.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  unbiasing_filter
  PRIVATE "UNBIASING_FILTER_BUILDING_LIBRARY"
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES unbiasing_filter.xml
        DESTINATION share/unbiasing_filter
)

install(TARGETS unbiasing_filter
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

# Tests
if(BUILD_TESTING)
  # Find dependencies
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  set(THIS_TEST_PACKAGE_INCLUDE_DEPENDS
      rclcpp
      geometry_msgs
      trajectory_msgs
      tf2_geometry_msgs
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    if(NOT ${dependency}_FOUND)
      find_package(${dependency} REQUIRED)
    endif()
  endforeach()

  # Add tests
  set(TEST_NAME
      ${PROJECT_NAME}_test
  )

  ament_add_gtest(${TEST_NAME}
                  test/test_unbiasing_filter.cpp
                  TIMEOUT
                  240
  )

  ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  target_link_libraries(${TEST_NAME} unbiasing_filter)

  ament_add_gtest_executable(gravity_compensation_fixed_orientation_test
                             test/test_gravity_compensation_fixed_orientation.cpp
  )

  add_ros_test(test/test_gravity_compensation_fixed_orientation.py)

  ament_target_dependencies(gravity_compensation_fixed_orientation_test
                            ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )

  target_link_libraries(gravity_compensation_fixed_orientation_test unbiasing_filter)

  install(DIRECTORY launch
          DESTINATION share/${PROJECT_NAME}/
  )

  install(DIRECTORY test/config
          DESTINATION share/${PROJECT_NAME}/test
  )

  # Install test files
  install(TARGETS ${TEST_NAME} gravity_compensation_fixed_orientation_test
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_dependencies(${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
