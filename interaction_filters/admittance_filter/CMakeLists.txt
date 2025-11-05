cmake_minimum_required(VERSION 3.8)
project(admittance_filter)

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
    interaction_filter_base
    pluginlib
    Eigen3
    generate_parameter_library
    acg_common_libraries
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(${EIGEN3_INCLUDE_DIR})

# Generate the parameter library
generate_parameter_library(admittance_filter_parameters src/admittance_filter_parameters.yaml)

# Shared library
add_library(admittance_filter SHARED
            src/admittance_filter.cpp
)

target_include_directories(
  admittance_filter
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

target_link_libraries(admittance_filter admittance_filter_parameters)

ament_target_dependencies(admittance_filter ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(interaction_filter_base admittance_filter.xml)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  admittance_filter
  PRIVATE "ADMITTANCE_FILTER_BUILDING_LIBRARY"
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES admittance_filter.xml
        DESTINATION share/admittance_filter
)

install(TARGETS admittance_filter admittance_filter_parameters
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
      tf2_geometry_msgs
      std_msgs
      acg_common_libraries
      acg_control_msgs
      rosbag2_cpp
  )

  foreach(dependency IN
          ITEMS ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS}
  )
    if(NOT ${dependency}_FOUND)
      find_package(${dependency} REQUIRED)
    endif()
  endforeach()

  ament_add_gtest(admittance_filter_test test/test_admittance_filter.cpp)

  ament_target_dependencies(admittance_filter_test ${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})

  target_link_libraries(admittance_filter_test admittance_filter)

  # Install test files
  install(TARGETS admittance_filter_test
          RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY test/bagfiles
          DESTINATION share/${PROJECT_NAME}/test
  )

  ament_export_dependencies(${THIS_TEST_PACKAGE_INCLUDE_DEPENDS})
endif()

execute_process(COMMAND rm doc/admittance_filter_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                ERROR_QUIET
)
execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                        src/admittance_filter_parameters.yaml --output_markdown_file
                        doc/admittance_filter_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
