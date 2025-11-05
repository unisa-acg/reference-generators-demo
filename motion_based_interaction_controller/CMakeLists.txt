cmake_minimum_required(VERSION 3.16)
project(motion_based_interaction_controller)

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
    acg_common_libraries
    acg_control_msgs
    acg_hardware_interface_facade
    controller_interface
    Eigen3
    filters
    generate_parameter_library
    geometry_msgs
    interaction_filter_base
    kinematics_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    tf2_eigen
)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

# Generate the parameter library
generate_parameter_library(motion_based_interaction_controller_parameters
                           src/motion_based_interaction_controller_parameters.yaml
)

# Shared library
add_library(motion_based_interaction_controller SHARED
            src/motion_based_interaction_controller.cpp
)

target_include_directories(
  motion_based_interaction_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/motion_based_interaction_controller>
)

target_link_libraries(motion_based_interaction_controller
                      motion_based_interaction_controller_parameters
)

ament_target_dependencies(motion_based_interaction_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  motion_based_interaction_controller
  PRIVATE "MOTION_BASED_INTERACTION_CONTROLLER_BUILDING_DLL"
)

install(DIRECTORY include/
        DESTINATION include/motion_based_interaction_controller
)

install(TARGETS motion_based_interaction_controller motion_based_interaction_controller_parameters
        EXPORT export_motion_based_interaction_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

execute_process(COMMAND rm doc/motion_based_interaction_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                ERROR_QUIET
)
execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                        src/motion_based_interaction_controller_parameters.yaml
                        --output_markdown_file doc/motion_based_interaction_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
