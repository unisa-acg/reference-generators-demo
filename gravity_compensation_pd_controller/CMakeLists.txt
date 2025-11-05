cmake_minimum_required(VERSION 3.8)
project(gravity_compensation_pd_controller
        LANGUAGES CXX
)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
    acg_common_libraries
    acg_control_msgs
    acg_hardware_interface_facade
    controller_interface
    Eigen3
    generate_parameter_library
    hardware_interface
    inverse_dynamics_solver
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(gravity_compensation_pd_controller_parameters
                           src/gravity_compensation_pd_controller_parameters.yaml
)

add_library(gravity_compensation_pd_controller SHARED
            src/gravity_compensation_pd_controller.cpp
)

target_compile_features(
  gravity_compensation_pd_controller
  PUBLIC cxx_std_17
)

target_include_directories(
  gravity_compensation_pd_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/gravity_compensation_pd_controller>
)

target_link_libraries(
  gravity_compensation_pd_controller
  PUBLIC gravity_compensation_pd_controller_parameters
)

ament_target_dependencies(
  gravity_compensation_pd_controller
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  gravity_compensation_pd_controller
  PRIVATE "GRAVITY_COMPENSATION_PD_CONTROLLER_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(
  controller_interface gravity_compensation_pd_controller.xml
)

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(TARGETS gravity_compensation_pd_controller gravity_compensation_pd_controller_parameters
        EXPORT export_gravity_compensation_pd_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

execute_process(COMMAND rm doc/gravity_compensation_pd_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                ERROR_QUIET
)
execute_process(COMMAND generate_parameter_library_markdown --input_yaml
                        src/gravity_compensation_pd_controller_parameters.yaml
                        --output_markdown_file doc/gravity_compensation_pd_controller_parameters.md
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

ament_export_targets(export_gravity_compensation_pd_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
