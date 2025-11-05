cmake_minimum_required(VERSION 3.8)
project(moving_average_filter)

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
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Shared library
add_library(moving_average_filter SHARED
            src/moving_average_filter.cpp
)

target_include_directories(
  moving_average_filter
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(moving_average_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(filters moving_average_filter.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  moving_average_filter
  PRIVATE "MOVING_AVERAGE_FILTER_BUILDING_LIBRARY"
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES moving_average_filter.xml
        DESTINATION share/moving_average_filter
)

install(TARGETS moving_average_filter
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
      rclcpp filters
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
                  test/test_moving_average_filter.cpp
                  TIMEOUT
                  240
  )

  ament_target_dependencies(${TEST_NAME} ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  target_link_libraries(${TEST_NAME} moving_average_filter)

  # Install test files
  install(TARGETS ${TEST_NAME}
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_dependencies(${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
