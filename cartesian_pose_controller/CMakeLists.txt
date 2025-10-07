cmake_minimum_required(VERSION 3.8)
project(cartesian_pose_controller)

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

set(CARTESIAN_POSE_CONTROLLER_INCLUDE_DEPENDS
    acg_control_msgs
    controller_interface
    generate_parameter_library
    geometry_msgs
    pluginlib
    kinematics_interface
    acg_hardware_interface_facade
    acg_common_libraries
    rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dependency IN
        ITEMS ${CARTESIAN_POSE_CONTROLLER_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(cartesian_pose_controller_parameters
                           src/cartesian_pose_controller_parameters.yaml
)

add_library(cartesian_pose_controller SHARED
            src/cartesian_pose_controller.cpp
)

target_compile_features(
  cartesian_pose_controller
  PUBLIC cxx_std_17
)

target_include_directories(
  cartesian_pose_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/cartesian_pose_controller>
)

target_link_libraries(
  cartesian_pose_controller
  PUBLIC cartesian_pose_controller_parameters
)

ament_target_dependencies(
  cartesian_pose_controller
  PUBLIC
  ${CARTESIAN_POSE_CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  cartesian_pose_controller
  PRIVATE "CARTESIAN_POSE_CONTROLLER_BUILDING_DLL"
)
pluginlib_export_plugin_description_file(controller_interface cartesian_pose_controller.xml)

install(DIRECTORY include/
        DESTINATION include/cartesian_pose_controller
)

install(TARGETS cartesian_pose_controller cartesian_pose_controller_parameters
        EXPORT export_cartesian_pose_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

execute_process(COMMAND rm doc/cartesian_pose_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                ERROR_QUIET
)
execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                        src/cartesian_pose_controller_parameters.yaml --output_markdown_file
                        doc/cartesian_pose_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

ament_export_targets(export_cartesian_pose_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${CARTESIAN_POSE_CONTROLLER_INCLUDE_DEPENDS})
ament_package()
